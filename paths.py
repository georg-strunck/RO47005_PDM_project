from controller import PDcontrolller
# from traject_fn import tj_from_line, tj_from_c, tj_from_c2, position_from_angle2
from traject_fn import tj_from_line
import numpy as np
import csv


def rrtstar_path(t):
    with open('final_path.csv') as f:
        reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        data = list(reader)
        #print(data[1:])
        data = data[1:]
        
    T=30
    vel = [0,0,0]
    pos = [0,0,0]
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
    
    # onderstaande nog fixen zodat start positie eerste CSV point is
    # if (t<T-0.01):
    #     seg_id_float = progress*num_segms   # where we are in the segments
    #     seg_id = np.floor(seg_id_float)     # round down --> this is the position we currently started from going to the next
    #     seg_id = int(seg_id+1)
    #     #print(seg_id)
    # else:
    #     seg_id = num_segms
    #     #print(seg_id)

    # if (t < segm_time):     # if we are not yet at the starting position data[0], go from pos to data[0]
    #     pos, vel = tj_from_line(pos,data[1],segm_time,t)
    #     #print(t)
    #     print(seg_id)
    
    # elif (t < seg_id*segm_time):  # if time smaller than T (end time) then update position based on where we are now (seg_id)
    #     pos, vel = tj_from_line(data[seg_id-1], data[seg_id], segm_time, t - segm_time*seg_id)
    #     #print(seg_id)
    # else:       # else stay at the last position in data
    #     pos = data[-1]  
            
    desired_state = dict([
        ('x', pos),  
        ('x_dot', vel),
        ('x_ddot',acc),
        ('yaw', yaw),
        ('yaw_dot', yawdot)
        ])
    return desired_state

