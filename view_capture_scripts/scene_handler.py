import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from abc import ABC, abstractmethod

class SceneHandler(ABC):
    @abstractmethod
    def add_object(obj_type: str, obj_name: str, obj_position: Point, 
                   obj_size: list, attachment_to_transform: str=None, file_name: str=None, 
                   planning_frame: str="world"):
        return None

    @abstractmethod
    def attach_obj(attachment_joint, obj_name):
        return None

    @abstractmethod
    def remove_object_from_scene(self, obj_name):
        return None

class MoveitSceneHandler(SceneHandler):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.scene = moveit_commander.PlanningSceneInterface()

    def add_object(self, obj_type: str, obj_name: str, obj_position: Point, 
                         obj_size: list, attachment_to_transform: str=None, attach:bool=False, 
                         file_name: str=None, planning_frame: str="world"):
        
        # Creates object position in the scene
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = planning_frame
        obj_pose.pose.position = obj_position
        obj_pose.pose.orientation.w = 1.0
    
        # Adds specific object to the scene of a specific object type
        if obj_type == "box":
            self.scene.add_box(obj_name, obj_pose, obj_size)
        elif obj_type == "sphere":
            self.scene.add_sphere(obj_name, obj_pose, radius=obj_size[0])
        elif obj_type == "mesh":
            self.scene.add_mesh(obj_name, obj_pose, file_name, size=obj_size)
        else:
            raise Exception("Object type " + obj_type + " is not currently supported")

        rospy.sleep(0.1)

        # Attaches object to the robot's attachement
        if attach:
            attach_obj(attachment_joint, obj_name)

    def attach_obj(attachment_joint, obj_name):
        self.scene.attach_box(attachment_joint, obj_name)

    # Removes an object with a specified name from the scene
    def remove_object_from_scene(self, obj_name):
        self.scene.remove_world_object(obj_name)

        success = self.ensure_collision_update(obj_name)

        if not success:
            raise Exception("Could not remove object " + obj_name + " from scene")

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

     # Adds box to scene
    def add_box(self, obj_name: str, obj_position: Point, 
                         obj_size: list, attach: bool=False):
        self.add_object("box", obj_name, obj_position, obj_size, attach=attach)

    # Adds sphere to scene
    def add_sphere(self, obj_name: str, obj_position: Point, 
                            obj_radius:float, attach: bool=False):
        self.add_object("sphere", obj_name, obj_position, [obj_radius], attach=attach)

    # Adds mesh to scene
    def add_mesh(self, obj_name: str, obj_position: Point, obj_size: list, 
                          mesh_file_name: str, attach: bool=False):
        self.add_object("mesh", obj_name, obj_position, obj_size, 
                              file_name=mesh_file_name, attach=attach)

def get_scene_handler(scene_handler):
	if scene_handler == "moveit":
		return MoveitSceneHandler()

		""" ADD YOUR METHOD HERE """
		
	else:
		raise Exception(f"Unknown scene handler type: {scene_handler}")