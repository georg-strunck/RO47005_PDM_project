"""
The code in this file is made for the purpose of this project
"""

from controller import PDcontrolller
from matplotlib import pyplot as plt
from rrt_star_3D import RRTStar
from rrt_3D import RRT
import numpy as np
import csv
from itertools import product, combinations

class Create_path:
    """
    This class is able to create a environment from two csv files containing the bboxes and centers of the objects. 
    It can then create a global path using either RRT_star or RRT with the function find_path_rrt_star
    The function rrtstar_path devides this path in smaller steps and returns a dict with the desired state 
    for a drone on this path at a certain timestep. 
    
    bbox_file_name should be a string with the name of the csv file containing the bounding boxes of the objects
    centers_file_name should be a string with the name of the csv file containing the center positions of the objects
    start should be a [x,y,z] coordinate within the environment
    end should be a [x,y,z] coordinate within the environment
    When use_star is false it uses the regular RRT method, when use_star is True it uses the RRT_star method. Default is True.
    """
    def __init__(self, bbox_file_name, centers_file_name, start, end, use_star=True):
        
        self.use_star = use_star
        self.bbox_file_name = bbox_file_name
        self.centers_file_name = centers_file_name
        self.start = start
        self.end = end
        
        self.obstacle_list = self.load_object_list()
        self.play_area = self.play_area_from_objects()
           
    def load_object_list(self):
        """
        This function reads both the bboxes csv file and the center locations csv file. 
        It then creates a list from these objects with the following structure:
        [[x, y, z, x_width, y_width, z_height],....]
        """
        box_list = []
        cent_list = []
        with open(self.bbox_file_name, 'r') as file:
            boxes = csv.reader(file)
            for row_box in boxes:
                boxs = []
                for box in row_box:
                    boxs.append(float(box))
                box_list.append(boxs)        
        with open(self.centers_file_name,'r') as file:
            center = csv.reader(file)
            for row_cent in center:
                cents = []
                for cent in row_cent:
                    cents.append(float(cent))
                cent_list.append(cents)
        
        objects_list = list(map(list.__add__, cent_list, box_list))
        return objects_list
    
    def play_area_from_objects(self):
        """
        This function uses the obstacles to create a play_area which is 10 meters wider on each side 
        then the maximum position of the objects in the play_area. It returns a list in the format:
        [x_min_play_area, x_max_play_area, y_min_play_area, y_max_play_area, z_min_play_area, z_max_play_area]
        """
        
        x_min_play_area = 0
        x_max_play_area = 0
        y_min_play_area = 0
        y_max_play_area = 0
        z_min_play_area = 0
        z_max_play_area = 0
    
        for objects in self.obstacle_list:
            x_min_object = objects[0]-objects[3]/2
            x_max_object = objects[0]+objects[3]/2
            y_min_object = objects[1]-objects[4]/2
            y_max_object = objects[1]+objects[4]/2
            z_min_object = objects[2]-objects[5]/2
            z_max_object = objects[3]+objects[5]/2
            if x_min_object < x_min_play_area:
                x_min_play_area = x_min_object
            if y_min_object < y_min_play_area:
                y_min_play_area = y_min_object
            if z_min_object < z_min_play_area:
                z_min_play_area = z_min_object
            if x_max_object > x_max_play_area:
                x_max_play_area = x_max_object
            if y_max_object > y_max_play_area:
                y_max_play_area = y_max_object
            if z_max_object > z_max_play_area:
                z_max_play_area = z_max_object
        return ([x_min_play_area - 10, x_max_play_area + 10, y_min_play_area - 10, y_max_play_area + 10, z_min_play_area, z_max_play_area + 10])
    

    def final_path_plot(self, path, drone_trajectory):
        """
        This function uses the environment, the path given and the drone_trajectory given to 
        generate a final path from the environment and the simulated drone_trajectory.
        It draws all the obstacles as green boxes. 
        It draws the play area as black lines 
        It draws the simulated drone_trajectory as a blue line
        It draws the path from the global planner as red line
        """
        if path == None:
            print("helaas pindakaas")
        else: 
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            for i in range(len(self.obstacle_list)):
                x = [self.obstacle_list[i][0]-self.obstacle_list[i][3]/2, self.obstacle_list[i][0]+self.obstacle_list[i][3]/2]
                y = [self.obstacle_list[i][1]-self.obstacle_list[i][4]/2, self.obstacle_list[i][1]+self.obstacle_list[i][4]/2]
                z = [self.obstacle_list[i][2]-self.obstacle_list[i][5]/2, self.obstacle_list[i][2]+self.obstacle_list[i][5]/2]
                for s, e in combinations(np.array(list(product(x, y, z))), 2):
                    ax.plot3D(*zip(s, e), color="green")
                
            ax.scatter3D(self.start[0], self.start[1], self.start[2], color ='green')
            ax.scatter3D(self.end[0], self.end[1], self.end[2],  color = 'red')
            
            if self.play_area is not None:
                plt.plot([self.play_area[0], self.play_area[1],
                          self.play_area[1], self.play_area[0],
                          self.play_area[0]],
                         [self.play_area[2], self.play_area[2],
                          self.play_area[3], self.play_area[3],
                          self.play_area[2]],
                         "-k")
                ax.plot3D([self.play_area[0],self.play_area[0]],[self.play_area[2],self.play_area[2]],[self.play_area[4],self.play_area[5]], color="black")
            
            #for coor in range(len(path)-1):
                #ax.plot3D([path[coor][0],path[coor+1][0]],[path[coor][1],path[coor+1][1]],[path[coor][2],path[coor+1][2]], color="red")
            
            ax.plot3D(drone_trajectory['x'],drone_trajectory['y'],drone_trajectory['z'], 'blue')
    
            plt.show()
       
    def tj_from_line(self, start_pos, end_pos, time_ttl, t_c):
        """
        This function is used to calculate the desired position and velocity over a certain start and end position of
        a line segment of the path. 
        """
        
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        v_avg = (end_pos-start_pos)/time_ttl
        vel = v_avg
        pos = end_pos - (time_ttl-t_c)*v_avg
        acc = [0,0,0]
        return pos, vel
    
    def calc_path_length(self, path):
        """
        This function calculates the length of a path that is found by the RRT algorithm. 
        """
        total_length = 0
        for coor in range(len(path)-1):
            start = np.array(path[coor])
            end = np.array(path[coor+1])
            total_length += np.sqrt(np.sum((end-start)**2))
        return total_length
    
    def find_path_rrt_star(self):
        """
        This function takes the defined environment and returns a path in this environment.
        It can either use the RRT_star or RRT method define in the rrt_star_3D and rrt_3D files respectivly
        If it cannot find a path it returns None.
        """
        plt.close('all');
        print("start " + __file__)
        
        # Set Initial parameters
        if self.use_star:
            print("looking for a path with RRT_star")
            rrt_star = RRTStar(
                 start = self.start,
                 goal = self.end, 
                 play_area = self.play_area,
                 obstacle_list= self.obstacle_list,
                 expand_dis=1.0,
                 path_resolution=0.1,
                 goal_sample_rate=10,
                 max_iter=5000,
                 margin = 2,
                 connect_circle_dist=60.0,
                 search_until_max_iter=True,
                 animation = False)
            path = rrt_star.planning()
        else:
            print("looking for a path with RRT")
            rrt = RRT(
                start=self.start,
                goal= self.end,
                obstacle_list=self.obstacle_list,
                play_area=self.play_area,
                expand_dis=6.0,
                path_resolution=0.1,
                goal_sample_rate=20,
                max_iter=500,
                margin = 2,
                animation = False
                )
            path = rrt.planning()
        
        if path is None:
            print("Cannot find path")
            return None
        elif self.use_star:
            print("found path!!")
            rrt_star.final_path_plot(path)
            return path
        else:
            print("found path!!")
            rrt.final_path_plot(path)
            return path
            
    
    def rrtstar_path_state(self, t, path, sim_time):  
        """
        This function takes a path with certain checkpoints as an input and a certain time
        It then outputs a desired state for that time and the position it the drone should have on this path
        This can be used in the simulation of the drone. 
        """
        data = list(path)
            
        T=sim_time
        vel = [0,0,0]
        pos = data[0]
    
        acc = [0,0,0]
        yaw = 0
        yawdot = 0
        
        # The number of segments (lines)
        num_segms=len(data)
      
        #time needed per line segment 
        segm_time = T/num_segms
        
        progress = t/T          # percentage of where we are
        if (t<T-0.01):
            seg_id_float = progress*num_segms   # where we are in the segments
            seg_id = np.floor(seg_id_float)     # round down --> this is the position we currently started from going to the next
            seg_id = int(seg_id+1)
        else:
            seg_id = num_segms
    
        if (t < segm_time):     # if we are not yet at the starting position data[0], go from pos to data[0]
            pos, vel = self.tj_from_line(pos,data[0],segm_time,t)
        
        elif (t < seg_id*segm_time):  # if time smaller than T (end time) then update position based on where we are now (seg_id)
            pos, vel = self.tj_from_line(data[seg_id-2], data[seg_id-1], segm_time, t - segm_time*(seg_id-1))
        else:       # else stay at the last position in data
             pos = data[-1]
                       
        desired_state = dict([
            ('x', pos),  
            ('x_dot', vel),
            ('x_ddot',acc),
            ('yaw', yaw),
            ('yaw_dot', yawdot)
            ])
        return desired_state