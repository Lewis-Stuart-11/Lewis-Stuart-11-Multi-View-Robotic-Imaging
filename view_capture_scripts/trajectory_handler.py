from math import pi, sin, cos, sqrt
import matplotlib.pyplot as plt
import numpy as np 
import geometry_msgs.msg
import os
from mpl_toolkits.mplot3d import Axes3D

# Handles all potential points to be traversed by the robot
class TrajectoryHandler(object):
    def __init__(self, X_OFFSET, Y_OFFSET, MAX_DIST, MIN_Z, log_dir="", experiment_name=""):
        self.all_positions = []

        # Stores which points are predicted to be traversable 
        self.predicted_positions = {"valid_positions": {}, "invalid_positions": {}}

        # Stores points which were successfully and unsucessfully traversed 
        self.traversed_positions = {"valid_positions": {}, "invalid_positions": {}}

        # Determine which points are traversable
        self.pos_dist = lambda pos: sqrt(pos.x**2 + pos.y**2 + pos.z**2)
        self.is_pos_outside_centre = lambda pos: (pos.x > X_OFFSET or pos.x < -X_OFFSET) or (pos.y > Y_OFFSET or pos.y < -Y_OFFSET)
        self.is_pos_outside_boundaries = lambda pos: (self.pos_dist(pos) < MAX_DIST) and (pos.z > MIN_Z)
        self.is_pos_valid = lambda pos: self.is_pos_outside_centre(pos) and self.is_pos_outside_boundaries(pos)

        # Stores the current index 
        self.current_point_idx = 0
        self.current_failed_point_idx = 0
        self.current_sect_idx = 0

        self.log_dir = log_dir
        self.experiment_name = experiment_name

        self.sectors = None
        self.rings = None

        self.point_matrix = []

        self.points_in_sectors = {}

    # Calculates a series of points in a spherical pattern around an origin
    def calculate_sphere_points(self, sphere_origin, radius, rings=10, sectors=20, 
                                cut_off=0.3, aabb=None):
        self.rings = rings
        self.sectors = sectors
        
        self.point_matrix = np.full((sectors, rings), -1)

        ring_d = 1.0/(rings)
        sect_d = 1.0/(sectors)

        # Cut off results in a 'half' sphere being generated, meaning points 'too low' will
        # not be included in the calculations
        ring_d *= (1-cut_off)

        # Adds point directly above the origin at a distance of the radius
        """if aabb is not None:
            if aabb[5] >= 1:
                self.add_point(0, -0.0000000001, 1, sphere_origin, radius)"""

        #self.test(sphere_origin, radius)

        # Loops through every sector and ring and calculates a new point
        for sect in range(sectors):
            for ring in range(1, rings+1, 1):
                
                # Ensures that on next sector, the ring is on the same level as the previous (produces a smother trajectory)
                if sect % 2 == 1:
                    ring = rings-ring + 1

                x = cos(2 * pi * sect * sect_d) 
                y = sin(2 * pi * sect * sect_d) 
                
                x *= sin(pi * ring * ring_d)
                y *= sin(pi * ring * ring_d)

                z = -sin(-(pi/2) + (pi * ring * ring_d))

                if aabb is not None:
                    if x < aabb[0] or x > aabb[1]:
                        continue
                    if y < aabb[2] or y > aabb[3]:
                        continue
                    if z < aabb[4] or z > aabb[5]:
                        continue

                self.point_matrix[sect][ring-1] = len(self.all_positions)

                self.add_point(x, y, z, sphere_origin, radius)

                self.points_in_sectors[str(len(self.all_positions)-1)] = sect

    
    def test(self, sphere_origin, radius):
        #difference = 0.0000000001

        #self.add_point(difference, difference, 1, sphere_origin, radius)
        #self.add_point(difference, -difference, 1, sphere_origin, radius)
        #self.add_point(-difference, difference, 1, sphere_origin, radius)
        #self.add_point(-difference, -difference, 1, sphere_origin, radius)

        circle_radius = 0.03

        num_points = 8

        for i in range(num_points):
            angle = (2 * pi) * (i/num_points)

            x = circle_radius * cos(angle)
            y = circle_radius * sin(angle)

            self.add_point(x, y, 0.99, sphere_origin, radius)

    # Adds a new point to traverse and checks if the point is valid
    def add_point(self, x, y, z, sphere_origin, radius, validate_point=True):

        new_point = geometry_msgs.msg.Point()

        # Point position is multiplied by the capture radius to ensure it is the 
        # correct distance from the origin
        new_point.x = x * radius
        new_point.y = y * radius
        new_point.z = z * radius

        # Point position is translated relative to the origin
        new_point.x += sphere_origin.x
        new_point.y += sphere_origin.y
        new_point.z += sphere_origin.z 
        
        self.all_positions.append(new_point)

        # The point is tested to see if it valid and added to the appropriate dictionary key list
        if validate_point:
            if self.is_pos_valid(new_point):
                self.predicted_positions["valid_positions"][len(self.all_positions)-1] = new_point
            else:
                self.predicted_positions["invalid_positions"][len(self.all_positions)-1] = new_point
        else:
            self.predicted_positions["valid_positions"][len(self.all_positions)-1] = new_point
    
    def get_adjacent_position_for_point(self, position_idx, pos_range=2):
        matrix_idxs = np.where(self.point_matrix == position_idx)

        pos_idx_x = matrix_idxs[0][0]
        pos_idx_y = matrix_idxs[1][0]

        adjacent_positions = []

        for i in range(-pos_range, pos_range+1):
            for j in range(-pos_range, pos_range+1):
                if i == 0 and j == 0:
                    continue
                
                adjacent_idx_x = (pos_idx_x + i) % self.sectors
                adjacent_idx_y = (pos_idx_y + j) 

                if adjacent_idx_y < 0 or adjacent_idx_y >= self.rings:
                    continue

                #adjacent_idx = ((adjacent_idx_x * self.sectors) + adjacent_idx_y) % len(self.all_positions)

                adjacent_pos = self.point_matrix[adjacent_idx_x][adjacent_idx_y]

                if adjacent_pos != -1:
                    adjacent_positions.append(adjacent_pos)

        return adjacent_positions
    
    def get_adjacent_successful_positions(self, img_pair_range=2):
        
        adjacencies_for_all_points = {}
        
        for pos_idx, _ in enumerate(self.all_positions):
            if pos_idx not in self.traversed_positions["valid_positions"].keys():
                continue

            adjacent_points = self.get_adjacent_position_for_point(pos_idx, pos_range=img_pair_range)

            valid_points = set()

            for adjacent_point in adjacent_points:
                if adjacent_point in self.traversed_positions["valid_positions"]:
                    valid_points.add(adjacent_point)

            adjacencies_for_all_points[pos_idx] = valid_points

        return adjacencies_for_all_points
        
    def visualise_predicted_valid_points(self, save=False):
        self.visualise_points(self.predicted_positions, save=save, predicted_points=True)

    def visualise_traversed_points(self, save=False):
        self.visualise_points(self.traversed_positions, save=save)

    # Visualises a series of positions by generating a 3D coloured scatter graph
    def visualise_points(self, positions, show_order=False, save=False, 
                         predicted_points=False, show_points_in_space=True):
        
        # Properties for the different types of points to visualise
        visualisation = {"valid_positions": {"colour": "b", "marker": "^"},
                         "invalid_positions": {"colour": "r", "marker": "o"}}

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        
        # Adds each point to the scatter diagram, and adds the correct colour and symbol 
        for point_type, points in positions.items():
            for i, point in points.items():
                ax.scatter(point.x, point.y, point.z, c=visualisation[point_type]["colour"], marker=visualisation[point_type]["marker"])
                if show_order:
                    ax.text(point.x, point.y, point.z, (str(i+1)), size=10, zorder=1, color=visualisation[point_type]["colour"])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        if show_points_in_space:
            ax.set_xlim3d(-1, 1)
            ax.set_ylim3d(-1, 1)
            ax.set_zlim3d(-1, 1)

        plt.title(self.experiment_name.replace("_", " ").capitalize())

        # Saves the figure if defined by the argument
        if save:
            save_fig_file = "Predicted_points" if predicted_points else "Traversed_points"

            save_fig_path = os.path.join(self.log_dir, save_fig_file + ".png")

            plt.savefig(save_fig_path)

        plt.show()

    # Generator that traverses all points that were generated by the spherical algorithm
    def get_next_positions(self):
        while self.current_point_idx < len(self.all_positions):
            current_index = self.current_point_idx
            current_pos = self.all_positions[current_index]  

            self.current_point_idx += 1

            yield current_index, current_pos

    # Generator that traverses all previously failed points
    def get_failed_positions(self):
        while self.current_failed_point_idx < len(self.traversed_positions["invalid_positions"]):

            current_pos = list(self.traversed_positions["invalid_positions"].values())[self.current_failed_point_idx]  
            current_index = list(self.traversed_positions["invalid_positions"].keys())[self.current_failed_point_idx]

            self.current_failed_point_idx += 1

            yield current_index, current_pos

    def get_next_sector(self, ignore_previously_traversed_positions=True):
        while self.current_sect_idx < self.sectors:
            sector_positions = []

            start_idx = self.current_point_idx

            for point_idx, sect in self.points_in_sectors.items():
                if sect != self.current_sect_idx:
                    continue

                if point_idx in self.traversed_positions["valid_positions"].keys() and ignore_previously_traversed_positions:
                    continue

                sector_positions.append(self.all_positions[int(point_idx)])
                self.current_point_idx += 1
            
            self.current_sect_idx += 1

            yield start_idx, sector_positions

    # Updates a points status (whether it was successfully traversed or not)
    def pos_verdict(self, current_pos_idx, success):
        current_pos = self.all_positions[current_pos_idx] 
        
        # If the point was successfully traversed, then add it to the successfully traversed list
        if success:
            self.traversed_positions["valid_positions"][current_pos_idx] = current_pos
            if current_pos_idx in self.traversed_positions["invalid_positions"].keys():
                del self.traversed_positions["invalid_positions"][current_pos_idx]
        
        # Otherwiuse add it to the invalid point list
        else:
            self.traversed_positions["invalid_positions"][current_pos_idx] = current_pos

    def reset_traversed_positions():
        self.current_sect_idx = 0
        self.current_point_idx = 0

    #def reset_all():

    # Saves all successfully and unsuccessfully traversed points as a CVS file
    def save_positions(self):
        with open(self.save_directory+"_points.txt", "w") as positions_file:
            positions_file.write("All Positions:\n")
            for idx, position in enumerate(self.all_positions):
                positions_file.write(str(idx) + ": " + str(position.x) + " " + str(position.y) + " " + str(position.z) + "\n")
            
            positions_file.write("\n")

            for pos_type, positions in self.traversed_positions.items():
                positions_file.write(pos_type.replace("_", " ").capitalize() + "\n")
                for idx, position in positions.items():
                    positions_file.write(str(idx) + ": " + str(position.x) + " " + str(position.y) + " " + str(position.z) + "\n")
            
                positions_file.write("\n")