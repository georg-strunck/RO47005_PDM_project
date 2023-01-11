from zmqRemoteApi import RemoteAPIClient
import numpy as np
import math 

def initialize_simulation():
    '''
    Initialize the simulation by connecting to the coppelia environment
    '''
    # Maybe add a sim.simxfinish here, so it can restart better on its own

    # Create Client
    client = RemoteAPIClient()
    print('Created Client at', client)
    # get sim class
    sim = client.getObject('sim')
    print('Got Object sim at', sim)
    # Set simulation time to real time (I think)
    client.setStepping(True)
    print('Set Stepping of client to True')
    # Start the simulation (equal to clicking play in coppelia, but then with the following code)
    sim.startSimulation()
    print('Started Simulation of sim')

    return client, sim

def get_pos_and_bbox(num_buil, sim):
    '''
    Getting the center position and bounding box size of 'column' objects up until num_buil
    num_buil = number of columns to check
    sim = the simulation class
    '''
    obj_buil = []           # list of handles for buildings
    for i in range(num_buil):
        current_obj = sim.getObject('./column['+str(i)+']')
        obj_buil.append(current_obj)

    pos_buil = []           # list of building objects position (x, y, z)
    bbox_bui = []           # list of bounding boxes per building shape/object --> length in x, y, z
    # Go through all handles of the objects
    for j in obj_buil:
        # Get position of current object/handle and append to pos_buil
        current_pos = sim.getObjectPosition(j, sim.handle_world)
        pos_buil.append(current_pos)
        # Get bbox size of current object/handle and append to bbox_buil
        current_bbox = sim.getShapeBB(j)
        bbox_bui.append(current_bbox)
    return pos_buil, bbox_bui

def get_position(quad_handle, sim):
    '''
    Returns the handle and current position of the specified object in the simulation
    sim =  the simulation class
    string = './Quadcopter/target' for example, is the name of object in the coppellia tree
    '''
    ## Get handle of quadcopter
    # quad_handle = sim.getObject(string)
    # Get position of quadcopter
    quad_pos = sim.getObjectPosition(quad_handle, sim.handle_world)

    return quad_pos

def calc_total_distance_xyz(arr_a, arr_b):
    arr_c = arr_a - arr_b
    total_distance = np.sqrt(sum(arr_c**2))
    return total_distance
    
def generate_drone_trajectory_from_path_array(traj_start, traj_max_stepsize, traj_path):
    '''
    Generates new path with steps not exceeding the max_stepsize criterion from a given path array.
    Every position in the specified path will be connected in a straight line
    start = the start position of the drone
    max_stepsize = the distance the drone will fly in one timestep of coppelia (0.05 [s])
    path = the path that was found with rrt
    
    Returns the new path as a list of waypoint position arrays
    '''
    # Make sure start is array
    traj_start = np.asarray(traj_start)
    # Create list to which to append the sub waypoints
    my_drone_path = []
    #Go through every waypoint of the given path and check if substeps are needed and append all steps to my_drone_path
    for subgoal in traj_path:
        # If we are not yet at the goal do:
        if (traj_start != subgoal).any():
            direction_vector = subgoal - traj_start                         # get the vector from a to b
            dist = calc_total_distance_xyz(subgoal, traj_start)             # get the distance between a and b
            n_steps = math.ceil(dist/traj_max_stepsize)                     # calculate the minimum amount of steps needed from a to b
            for progress in np.linspace(1/n_steps, 1, n_steps):
                new_subsubgoal = traj_start + progress * direction_vector   # Create new waypoint on straight line between a and b
                my_drone_path.append(new_subsubgoal)
        # If we are at the goal just append the goal, so not to miss it
        else:
            my_drone_path.append(subgoal)
        # Update the starting position before checking the next original path waypoint
        traj_start = subgoal
    return my_drone_path