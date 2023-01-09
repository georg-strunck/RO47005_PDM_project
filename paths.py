from controller import PDcontrolller
# from traject_fn import tj_from_line, tj_from_c, tj_from_c2, position_from_angle2
from traject_fn import tj_from_line
from matplotlib import pyplot as plt
from rrt_star_3D import RRTStar
import numpy as np
import csv
from itertools import product, combinations

def load_object_list(bboxes_file_name,centers_file_name):
    box_list = []
    cent_list = []
    with open('bboxes_objects.csv', 'r') as file:
        boxes = csv.reader(file)
        for row_box in boxes:
            boxs = []
            for box in row_box:
                boxs.append(float(box))
            box_list.append(boxs)        
    with open('center_positions_objects.csv','r') as file:
        center = csv.reader(file)
        for row_cent in center:
            cents = []
            for cent in row_cent:
                cents.append(float(cent))
            cent_list.append(cents)
    
    objects_list = list(map(list.__add__, cent_list, box_list))
    return objects_list

def play_area_from_objects(objects_list):
    x_min_play_area = 0
    x_max_play_area = 0
    y_min_play_area = 0
    y_max_play_area = 0
    z_min_play_area = 0
    z_max_play_area = 0

    for objects in objects_list:
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


def final_path_plot(path, obstacle_list, play_area, drone_trajectory, start, end):
    if path == None:
        print("helaas pindakaas")
    else: 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(len(obstacle_list)):
            x = [obstacle_list[i][0]-obstacle_list[i][3]/2, obstacle_list[i][0]+obstacle_list[i][3]/2]
            y = [obstacle_list[i][1]-obstacle_list[i][4]/2, obstacle_list[i][1]+obstacle_list[i][4]/2]
            z = [obstacle_list[i][2]-obstacle_list[i][5]/2, obstacle_list[i][2]+obstacle_list[i][5]/2]
            for s, e in combinations(np.array(list(product(x, y, z))), 2):
                ax.plot3D(*zip(s, e), color="green")
            
        ax.scatter3D(start[0], start[1], start[2], color ='green')
        ax.scatter3D(end[0], end[1], end[2],  color = 'red')
        
        if play_area is not None:
            plt.plot([play_area[0], play_area[1],
                      play_area[1], play_area[0],
                      play_area[0]],
                     [play_area[2], play_area[2],
                      play_area[3], play_area[3],
                      play_area[2]],
                     "-k")
            ax.plot3D([play_area[0],play_area[0]],[play_area[2],play_area[2]],[play_area[4],play_area[5]], color="black")
        
        #for coor in range(len(path)-1):
        #    ax.plot3D([path[coor][0],path[coor+1][0]],[path[coor][1],path[coor+1][1]],[path[coor][2],path[coor+1][2]], color="red")
        
        ax.plot3D(drone_trajectory['x'],drone_trajectory['y'],drone_trajectory['z'], 'blue')

        plt.show()

def find_path_rrt_star(objects_list, start, end, play_area):
    plt.close('all');
    print("start " + __file__)

    objects_list = objects_list# [[1.3499996662139893, -0.75, 0.5, 1.0, 1.0, 1.0], 
                    #[0.624998927116394, -3.4500019550323486, 0.5, 1.0, 1.0, 1.0], 
                    # [5.399999618530273, -3.599998712539673, 0.5, 1.0, 1.0, 1.0], 
                    # [7.849998474121094, -0.9999996423721313, 1.5, 3.0, 3.0, 3.0], 
                    # [-1.6499996185302734, -4.8499979972839355, 1.0, 4.0, 1.0, 2.0], 
                    # [3.97499942779541, 0.9000021815299988, 1.0, 4.0, 1.0, 2.0], 
                    # [-4.225001811981201, -1.4500010013580322, 2.0, 1.0, 5.0, 4.0], 
                    # [3.6499996185302734, -4.974996089935303, 1.0, 4.0, 1.0, 2.0], 
                    # [1.2500001192092896, 3.0000009536743164, 1.0, 4.0, 1.0, 2.0], 
                    # [-1.3749998807907104, 5.650001049041748, 1.0, 4.0, 1.0, 2.0], 
                    # [-4.700002193450928, 6.299997806549072, 2.0, 1.0, 5.0, 4.0]]
    # Set Initial parameters
    rrt_star = RRTStar(
         start = start, #start [7.5, -5, 0],
         goal = end, #[-6, 2.5, 1],
         play_area = play_area, # [-7, 10, -7, 10, 0, 6],
         obstacle_list=objects_list,
         expand_dis=6.0,
         path_resolution=0.1,
         goal_sample_rate=20,
         max_iter=500,
         margin = 2,
         connect_circle_dist=12.0,
         search_until_max_iter=False)
    path = rrt_star.planning()
    
    
    if path is None:
        print("Cannot find path")
        return None
    else:
        print("found path!!")
        rrt_star.final_path_plot(path)
        return path

def rrtstar_path(t, path):
    data = list(path)
        
    T=150
    vel = [0,0,0]
    pos = path[0]
    #pos = data[0]

    acc = [0,0,0]
    yaw = 0
    yawdot = 0
    
    # The number of segments (lines)
    num_segms=len(data)   # was len(data) - 1
    #num_segms=len(data)-1
  
    #time needed per line segment 
    segm_time = T/num_segms

    
    progress = t/T          # percentage of where we are
    #print(t)
    if (t<T-0.01):
        seg_id_float = progress*num_segms   # where we are in the segments
        seg_id = np.floor(seg_id_float)     # round down --> this is the position we currently started from going to the next
        seg_id = int(seg_id+1)
        #print(seg_id)
    else:
        seg_id = num_segms
        #print(seg_id)

    if (t < segm_time):     # if we are not yet at the starting position data[0], go from pos to data[0]
        pos, vel = tj_from_line(pos,data[0],segm_time,t)
        #print(t)
        #print(seg_id)
    
    elif (t < seg_id*segm_time):  # if time smaller than T (end time) then update position based on where we are now (seg_id)
        pos, vel = tj_from_line(data[seg_id-2], data[seg_id-1], segm_time, t - segm_time*(seg_id-1))
        #print(seg_id)
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

