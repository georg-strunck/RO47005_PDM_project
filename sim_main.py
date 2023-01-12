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


import time, sys
from sim_utilities import *
from operator import add    # needed for list addition with map()

# Initialize simulation
client, sim = initialize_simulation()

# Get building sizes and positions
num_buil = 13            # number of buildings
pos_buil, bbox_bui = get_pos_and_bbox(num_buil, sim)

# Export center positions of buildings
np.savetxt("center_positions_objects.csv", pos_buil, delimiter=",")
# Export bounding box sizes of buildings
np.savetxt("bboxes_objects.csv", bbox_bui, delimiter=",")

# Get handles of quadcopter target (green ball) and of quadcopter itself for later use
target_handle = sim.getObject('./Quadcopter/target')
quad_handle = sim.getObject('./Quadcopter')
# Current (starting) position of quadcopter target
target_position = get_position(target_handle, sim)

# Load path from csv file
path_from_csv = np.genfromtxt("coppelia_path.csv", delimiter=",")

# Set both the quadcopter and target to the starting position of the loaded path
startpoint_list = path_from_csv[0].tolist()
print(target_position, startpoint_list)
sim.setObjectPosition(target_handle, sim.handle_world, startpoint_list)
sim.setObjectPosition(quad_handle, sim.handle_world, startpoint_list)

# Get the starting position of the quadcopter
start = get_position(quad_handle, sim)

# Define the maximum stepsize of the target per time step (~ to speed)
max_stepsize = 0.08
# Generate the path with straight segments in between so not to destabilize the drone
my_drone_path_csv = generate_drone_trajectory_from_path_array(start, max_stepsize, path_from_csv)

# Run until time 40 
while (t := sim.getSimulationTime()) < 40:
    # Fly through path
    for waypoint in my_drone_path_csv:
        # Print current time
        try:
            print('Simulation time: ', round(sim.getSimulationTime(), 2), '  \t[s]')
            # Convert waypoint to list for coppelia sim
            waypoint_list = waypoint.tolist()
            # Update the target position to the waypoint (so the drone will fly towards it)
            sim.setObjectPosition(target_handle, sim.handle_world, waypoint_list)
            # Step the simulation forward (time step)
            client.step()
        # If keyboard interrupt, stop the simulation
        except KeyboardInterrupt:
            print('Ctrl+C was, pressed, exiting simulation ...')
            sim.stopSimulation()
    # Reverse path and fly back
    my_drone_path_csv = my_drone_path_csv[::-1]

# Stop the simulation
sim.stopSimulation()