"""
## How to run code:
        1. Open CoppeliaSim scene/environment
        2. Run this python script (it connects to Coppelia and starts the simulation)
## References/sources of code:
        Based on code snippet from:
        https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm
        zmqRemoteApi from:
        https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python
"""


import time
from my_utilities import *
from operator import add    # needed for list addition with map()

# Initialize simulation
client, sim = initialize_simulation()

# Get building sizes and positions
num_buil = 6            # number of buildings
pos_buil, bbox_bui = get_pos_and_bbox(num_buil, sim)

# Get handles of quadcopter target (green ball) and of quadcopter itself for later use
target_handle = sim.getObject('./Quadcopter/target')
quad_handle = sim.getObject('./Quadcopter')
# Current (starting) position of quadcopter target
target_position = get_position(target_handle, sim)


start = get_position(quad_handle, sim)
path = np.array([start,
                 [-1.75, -3., 0.5], 
                 [-1.6, -3.7, 3.], 
                 [-1.6, -6., 3.],
                 [-1.75, -7., 0.5],
                 [2.38, -5.7, 0.5],
                 start])
max_stepsize = 0.08
my_drone_path = generate_drone_trajectory_from_path_array(start, max_stepsize, path)

while (t := sim.getSimulationTime()) < 40:
    for waypoint in my_drone_path:
        print('Simulation time: ', round(sim.getSimulationTime(), 2), '\t[s]')
        waypoint_list = waypoint.tolist()
        sim.setObjectPosition(target_handle, sim.handle_world, waypoint_list)
        client.step()

sim.stopSimulation()


'''
# This code was just used during programming, but kept for now as backup
while (t := sim.getSimulationTime()) < 10:
    print('Simulation time: ', round(t, 2), '\t[s]')
    quadcopter_position = get_handle_and_pose(quad_handle, sim)

    dist_tar_quad = calc_total_distance_xyz(np.array(quadcopter_position), np.array(target_position))
    print(dist_tar_quad)
    print(quadcopter_position, target_position)
    target_position = get_handle_and_pose(target_handle, sim)
    new_position = list(map(add, target_position, [0.01, 0.01, 0.01])) 
    sim.setObjectPosition(target_handle, sim.handle_world, new_position)
    client.step()
'''