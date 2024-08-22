import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
import numpy as np

from quartonian_handler import QuartonianHandler

from urdf_parser_py.urdf import URDF
from collections import deque

import os

from copy import deepcopy

from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState

from sensor_msgs.msg import JointState

from abc import ABC, abstractmethod

class RobotHandler(ABC):
    @abstractmethod
    def set_endeffector_transform_name(self, transform_name: str):
        return None

    @abstractmethod
    def get_endeffector_transform_name(self):
        return None

    @abstractmethod
    def set_base_transform_name(self, transform_name: str):
        return None

    @abstractmethod
    def get_base_transform_name(self):
        return None

    @abstractmethod
    def set_camera_handler(self, camera_handler):
        return None

    @abstractmethod
    def get_camera_handler(self):
        return None

    @abstractmethod
    def set_camera_transform_name(self, transform_name: str):
        return None

    @abstractmethod
    def get_camera_transform_name(self):
        return None

    @abstractmethod
    def set_reach(self, reach):
        return None

    @abstractmethod
    def get_reach(self) -> float:
        return None

class RobotController(ABC):
    @abstractmethod
    def add_position(self, position, robot_idx):
        return None

    @abstractmethod
    def add_orientation(self, orientation:Quaternion, robot_idx: int):
        return None

    @abstractmethod
    def add_position_and_orientation(self, position:Pose, orientation:Quaternion, robot_idx: int):
        return None

    @abstractmethod
    def execute_plan(self):
        return None

    @abstractmethod
    def create_new_robot(self) -> int:
        return None

    @abstractmethod
    def get_robot_transform(self, transform_type, robot_idx: int) -> tuple([list, list]):
        return None


# Handles all communication with the virtual robot
class MoveItRobotController(RobotController):
    def __init__(self, group_name, num_move_attempts: int=3,
                        wait_time: float=0.75, planning_time: float=3.0, velocity: float = 0.05,
                        planning_algorithm: str=None, transform_joint: str="camera_lens"):

        # Initialises python moveit and ros 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5_group', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # Sets the path planning algorithm, if given, otherwise the default is used
        if planning_algorithm:
            move_group.set_planner_id(planning_algorithm)

        # Sets the max planning time before timeing out
        move_group.set_planning_time(planning_time)

        move_group.set_max_velocity_scaling_factor(velocity)

        #move_group.set_max_acceleration_scaling_factor(0.001)

        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.waypoints = []

        self.scene = scene
        self.robot = robot

        self.controller_name = None
        
        # The number of path planning attempts to make for each position
        self.num_move_attempts = num_move_attempts

        # Pause time after each command is issued
        self.wait_time = wait_time

        self.current_joint_info = None

        self.base_transform = None

        self.camera_handler = None

        self.robots = []

        self.has_pos = False

        # Subscribes to the joint_states topic to get the current robot state
        rospy.Subscriber("joint_states", JointState, self.update_joint_state)

        self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)

    def create_new_robot(self):
        self.robots.append(MoveItRobotHandler())

        return len(self.robots)-1
    
    def get_robot(self, idx):
        return self.robots[idx]

    def get_robot_size(self):
        return len(self.robots)

    # Returns if ros is currently active
    def get_is_ros_active(self) -> bool:
        return rospy.is_shutdown()

    # Returns current robot position
    def get_current_position(self) -> Pose:
        return self.move_group.get_current_pose().pose.position

    # Returns current quartonian rotation
    def get_current_orientation(self) -> Quaternion:
        return self.move_group.get_current_pose().pose.orientation

    # Returns current eulur roll, pitch and yaw
    def get_current_rpy(self):
        return self.move_group.get_current_rpy()

    # Updates the current robot's joint states 
    def update_joint_state(self, data: dict):
        self.current_joint_info = data

    # Attempts to move robot to position
    def move_robot(self, position, reset_orientation: bool=True) -> bool:
        pose = Pose()

        pose.position = position

        if reset_orientation:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
    
        return self.execute_new_pose(pose)

    # Attempts to alter the current robot rotation
    def reorient_arm(self, quartonian: Quaternion) -> bool:
        pose = Pose()

        pose.position = self.get_current_position()

        pose.orientation = quartonian

        return self.execute_new_pose(pose)

    # Attempts to move the robot to a new position with a new rotation
    def move_and_orientate_robot(self, position: Point, rotation: Quaternion) -> bool:

        pose = Pose()

        pose.position = position

        pose.orientation = rotation

        return self.execute_new_pose(pose)

    # Attempts to execute a new pose
    def execute_new_pose(self, pose: Pose) -> bool:
        
        # Attempts to move the robot to the new pose after a set 
        # number of attempts
        for i in range(self.num_move_attempts):
            self.move_group.set_pose_target(pose)

            success = self.move_group.go(wait=True)

            self.move_group.stop()

            self.move_group.clear_pose_targets()

            if success:
                return True

        return False

    def add_position(self, position:Pose, robot_id: int):
        pose = Pose()

        pose.position = position

        end_effector_transform = self.robots[robot_id].get_endeffector_transform_name()

        self.move_group.set_pose_target(pose, end_effector_link=end_effector_transform)

        self.has_pos = True

    def add_orientation(self, orientation:Quaternion, robot_id: int):
        pose = Pose()

        pose.orientation = orientation

        end_effector_transform = self.robots[robot_id].get_endeffector_transform_name()

        self.move_group.set_pose_target(pose, end_effector_link=end_effector_transform)

        self.has_pos = True

    def add_position_and_orientation(self, position:Pose, orientation:Quaternion, robot_id: int):
        pose = Pose()

        pose.position = position
        pose.orientation = orientation

        end_effector_transform = self.robots[robot_id].get_endeffector_transform_name()

        self.move_group.set_pose_target(pose, end_effector_link=end_effector_transform)

        self.has_pos = True

    def get_average_base_position(self):
        robot_base_positions = []

        for robot in self.robots:
            base_transform = self.get_transform(robot.get_base_transform_name())
            
            robot_base_positions.append(base_transform[0])
    
        print(robot_base_positions)

        return np.mean(np.array(robot_base_positions), axis=0)

    def get_robot_destination_positions(self, planed_trajectory, robot_links):

        # Ensure produced transforms are given in the WCS
        header = Header()
        header.frame_id = "world"

        # Set position to calulate as the last position in the trajectory
        robot_state = RobotState()
        robot_state.joint_state.name = planed_trajectory.joint_trajectory.joint_names
        robot_state.joint_state.position = planed_trajectory.joint_trajectory.points[-1].positions

        # Solve forward kinematics and return transforms
        try:
            fk_response = self.fk_service(header, robot_links, robot_state)

            return fk_response.pose_stamped

        except rospy.ServiceException as e:
            return None

    def point_in_camera_view(self, camera_position, camera_orientation, point, fov, image_resolution):

        focal_length = (image_resolution[0] / 2) / np.tan(fov / 2)

        # Transform the 3D point into camera coordinates
        point_camera_coords = np.dot(np.linalg.inv(camera_orientation), point - camera_position)

        # Avoids points close to cam being detected
        near_clip = 0.3 

        # Check if the point is within the camera's frustum
        if (point_camera_coords[2] < near_clip):
            return False

        # Project the point onto the image plane
        point_image_coords = (focal_length * point_camera_coords[:2]) / point_camera_coords[2]

        # Check if the projected point is within the image resolution
        if (-image_resolution[0] / 2 <= point_image_coords[0] <= image_resolution[0] / 2
            and -image_resolution[1] / 2 <= point_image_coords[1] <= image_resolution[1] / 2):
            return True

        return False

    def is_robot_in_cam(self, cam_transform, joint_transforms, camera_properties) -> bool:
        qh = QuartonianHandler()

        # Convert camera configuration to numpy array
        cam_translation = np.array([cam_transform.pose.position.x, cam_transform.pose.position.y, cam_transform.pose.position.z])
        cam_quart_list = [cam_transform.pose.orientation.x, cam_transform.pose.orientation.y, cam_transform.pose.orientation.z, cam_transform.pose.orientation.w]
        cam_orientation = np.array(qh.convert_quart_to_rotation_matrix(cam_quart_list))

        # Get essential camera parameters
        fov = camera_properties["camera_angle_x"]
        resolution = (camera_properties["w"], camera_properties["h"])
        
        prev_position = None

        # Estimated thickness of robot joints 
        thickness = 0.075
        
        # Number of points to generate per meter 
        steps_per_metre = 25

        # Iterate through each joint transform for the robot, and generate a series of points that connect each adjacent transform together
        # Check if any of the points are currently viewed by the camera, if so, then the current robot is most likely in the camera frame
        # Hence, a new path plan should be calculated
        for transform_i, current_translation in enumerate(joint_transforms):

            current_position = np.array([current_translation.pose.position.x, current_translation.pose.position.y, current_translation.pose.position.z])

            # Since a series of points are generated between two joint positions, a previous joint position needs to be traveresed first.
            # If two of the transforms have the same position, then no points should be generated 
            if prev_position is None or np.array_equal(current_position, prev_position):
                prev_position = current_position
                continue

            # Calculate the direction vector from the first joint position to the next joint position
            direction_vector = current_position - prev_position

            # Calculate the unit vector of the direction vector
            unit_direction = direction_vector / np.linalg.norm(direction_vector)

            # Calculate the normal vector 
            normal_vector = np.array([-unit_direction[1], unit_direction[0], 0])  # Adjust as needed

            # Calculates cross product between unit and normal vector
            cross_vector = np.cross(unit_direction, normal_vector)

            # Number of steps to take along the line between both joints (number of points to generate)
            steps_per_transform = int(np.linalg.norm(direction_vector) * steps_per_metre)+1

            # Amount to traverse pre step/point
            direction_vector_step = direction_vector * 1.0/float(steps_per_transform)

            robot_points = []

            # For each point calculate the points offset by the thickness perpendicular to the line in
            # both dimensions. Generates a square perpendicular to the current line direction
            # (top left, bottom left, top right, bottom right) 
            for i in range(steps_per_transform):

                current_trans = prev_position + (direction_vector_step * float(i)) 

                robot_points.append(current_trans + (thickness * normal_vector) + (thickness * cross_vector))
                robot_points.append(current_trans + (thickness * normal_vector) - (thickness * cross_vector))
                robot_points.append(current_trans - (thickness * normal_vector) + (thickness * cross_vector))
                robot_points.append(current_trans - (thickness * normal_vector) - (thickness * cross_vector))
            
            # Check if any of these points are inside of the camera's view point
            for robot_point in robot_points:
                if self.point_in_camera_view(cam_translation, cam_orientation, robot_point, fov, resolution):      
                    return True

            prev_position = current_position
        
        # The robot is not inside of the current camera frame
        return False

    def execute_plan(self, avoid_self_capture_paths=True) -> bool:

        if not self.has_pos:
            return False

        # Attempts to move the robot to the new poses after a set 
        # number of attempts
        for i in range(self.num_move_attempts):

            plan = self.move_group.plan()
            success = plan[0]
            planed_trajectory = plan[1]

            if avoid_self_capture_paths and success:
                for robot_idx, robot in enumerate(self.robots):
                    robot_links = robot.get_robot_links()

                    joint_transforms = self.get_robot_destination_positions(planed_trajectory, robot_links)

                    if joint_transforms is None:
                        print("Cannot solve forward kinematics for current plan, trying new configuration")

                    cam_transform = joint_transforms[-1]

                    camera_properties = robot.get_camera_handler().get_camera_properties()

                    if self.is_robot_in_cam(cam_transform, joint_transforms, camera_properties):
                        print(f"Current robot plan has robot {robot_idx} inside camera frame, trying new configuration")

                        success = False

                        break

            if not success:
                continue

            success = self.move_group.execute(planed_trajectory, wait=True)
        
            self.move_group.stop()

            if success:
                self.move_group.clear_pose_targets()

                self.has_pos = False

                return True

        self.move_group.clear_pose_targets()

        self.has_pos = False

        return False

    # Ensures that objects are correctly added to a scene in a given time
    def ensure_collision_update(self, obj_name: str, timeout: float=10) -> bool:
        start = rospy.get_time()
        current_time = rospy.get_time()

        while(current_time-start < timeout) and not rospy.is_shutdown():

            attached_objects = self.scene.get_attached_objects([obj_name])
            
            is_attached = len(attached_objects.keys()) > 0
            is_known = obj_name in self.scene.get_known_object_names()

            if is_attached and is_known:
                return True

            rospy.sleep(0.1)

            current_time = rospy.get_time() 

        return False

    def get_robot_transform(self, transform_type, robot_idx: int) -> tuple([list, list]):
        if transform_type == "end_effector":
            return self.get_transform(self.robots[robot_idx].get_endeffector_transform_name())
        elif transform_type == "camera":
            return self.get_transform(self.robots[robot_idx].get_camera_transform_name())
        elif transform_type == "base":
            return self.get_transform(self.robots[robot_idx].get_base_transform_name())
        else:
            raise Exception("Transform type {transform_type} does not currently exist for this type of robot")
    
    def get_transform(self, end_transform_joint, start_transform_joint="world"):
        num_tries = 3

        trans = None
        rot = None

        listener = tf.TransformListener()

        print(end_transform_joint)
        print(start_transform_joint)

        # Attempts a set number of times to retrieve the robot's transforms by querying the 
        # transform ros topic using the TF listener 
        for i in range(num_tries):
            try:
                (trans, rot) = listener.lookupTransform(f"/{start_transform_joint}", f"/{end_transform_joint}", rospy.Time(0))

                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(self.wait_time)
        
        if trans is None or rot is None:
            raise Exception("Unable to get transform for robot")

        return trans, rot
    
    # Retrieves the robot's joint information by querying the ROS joint state topic
    def get_current_joint_info(self):
        current_time = rospy.Time.now().to_sec()
        
        current_joint_state = None

        num_tries = 3

        # Attempts to retrieve the joint state information, each time checking that the time the 
        # joints were last updated is not earlier than the time the function was executed. 
        # This avoids outdated joint state information being returned
        for i in range(num_tries):
            joint_time = float(self.current_joint_info.header.stamp.to_time())

            if current_time - float(joint_time) > 0:
                rospy.sleep(self.wait_time)
            else:
                current_joint_state = self.current_joint_info
                break
        
        if current_joint_state is None:
            raise Exception("Unable to get joint state for robot")

        return current_joint_state


class URDFChainFinder:
    def __init__(self, urdf_file):
        self.robot = URDF.from_parameter_server()
        self.links = [link.name for link in self.robot.links]
        self.parent_map = {}
        self.build_parent_map()

    def build_parent_map(self):
        for joint in self.robot.joints:
            if joint.child in self.links:
                self.parent_map[joint.child] = joint.parent

    def find_chain(self, start_link, end_link):
        if start_link not in self.links or end_link not in self.links:
            return None  

        # Perform a DFS to find the chain
        visited = set()
        stack = deque()
        stack.append(start_link)
        chain = []

        while stack:
            current_link = stack.pop()
            visited.add(current_link)
            chain.append(current_link)

            if current_link == end_link:
                return chain  

            # Add unvisited child links to the stack
            for link, parent in self.parent_map.items():
                if parent == current_link and link not in visited:
                    stack.append(link)

        return None  

class MoveItRobotHandler(RobotHandler):

    def __init__(self):
        self.end_effector_transform = None
        self.end_base_transform = None
        self.end_camera_transform = None
        self.camera_handler = None
        self.reach = None
        self.urdf_file_path = None
        self.robot_links = None
        self.global_robot_id = None
    
    def set_global_id(self, robot_id):
        self.global_robot_id = robot_id
    
    def get_global_id(self):
        return self.global_robot_id

    def set_endeffector_transform_name(self, transform_name: str):
        self.end_effector_transform = transform_name

    def get_endeffector_transform_name(self):
        return self.end_effector_transform

    def set_base_transform_name(self, transform_name: str):
        self.end_base_transform = transform_name

    def get_base_transform_name(self) -> tuple([list, list]):
        return self.end_base_transform

    def set_camera_handler(self, camera_handler):
        self.camera_handler = camera_handler

    def get_camera_handler(self):
        return self.camera_handler

    def set_camera_transform_name(self, transform_name: str):
        self.end_camera_transform = transform_name

    def get_camera_transform_name(self):
        return self.end_camera_transform

    def set_reach(self, reach):
        self.reach = reach

    def get_reach(self) -> float:
        return self.reach

    def set_urdf_file_path(self, path):
        if not os.path.exists(path):
            raise Exception(f"URDF file path is not valid {path}")

        self.urdf_file_path = path 

    def get_robot_links(self):

        if self.robot_links is not None:
            return self.robot_links

        if self.end_effector_transform is None or self.end_base_transform is None:
            raise Exception("Robot must have a set base, end effector and valid URDF file path in order to determine link tree")

        if self.urdf_file_path is None:
            self.urdf_file_path = rospy.get_param("/robot_description") 

        chain_finder = URDFChainFinder(self.urdf_file_path)

        start_link = self.get_base_transform_name()
        end_link = self.get_endeffector_transform_name()     

        self.robot_links = chain_finder.find_chain(start_link, end_link)

        return self.robot_links

def get_robot_handler(robot_handler, group_name):
	if robot_handler == "moveit":
		return MoveItRobotController(group_name)

		""" ADD YOUR METHOD HERE """
		
	else:
		raise Exception(f"Unknown robot handler type: {robot_handler}")


