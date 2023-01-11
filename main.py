"""
The code in this file is made for the purpose of this by group 12
It is partly inspired by the code provided by the course RO47001 Robot Dynamics & Control from the TU Delft
"""

import gym
import numpy as np 
from controller import PDcontrolller
from paths import Create_path
import csv

#define the start and endpoint for the simulation
start = [-6, 2, 0]
end = [60, -120, 0]
#define the environment by the object csv files, the start and end postition and define wether to use RRTstar or RRT.
environment = Create_path('bboxes_objects.csv', 'center_positions_objects.csv', start, end, use_star=True)
#make a global path through the defined environment
#path = environment.find_path_rrt_star()

path = [[60, -120, 0],
 [60.07784972614862, -118.72865577737733, 1.9867626034737425],
 [60.473695707629865, -112.264194995032, 12.088943197407849],
 [60.86954168911068, -105.79973421268667, 22.191123791342026],
 [61.2653876705915, -99.33527343034135, 32.29330438527613],
 [61.46331066133191, -96.10304303916868, 37.34439468224339],
 [57.856114026938506, -94.71009301730467, 46.65437126103343],
 [54.832432145183866, -88.57555135264099, 56.11075826682567],
 [51.99726784730478, -83.4254416187533, 62.898333148566806],
 [47.97011962841585, -75.73697749415848, 66.4308859774829],
 [44.98069264645489, -72.01858632599223, 68.88578114736478],
 [37.038945196886935, -67.54782784196485, 71.3734885594309],
 [33.158732319525704, -64.39451040248363, 74.69019432754182],
 [24.218385860921128, -57.55127783506088, 75.97136173238114],
 [17.890342252325464, -49.10962097578382, 72.95457788747969],
 [14.803834109273861, -44.02949538084817, 73.77053831700694],
 [15.018177744539667, -37.730007862309996, 64.57376075980962],
 [17.737180443132043, -27.058228354438878, 64.17600501098549],
 [20.661235321631306, -16.638929496417116, 63.21592145407562],
 [22.10722768145038, -13.395022754402174, 58.37999598479298],
 [25.641050610698155, -8.468861334536935, 49.613801044937965],
 [29.841305538226646, -10.106923355757594, 40.23256778250653],
 [31.882502420311965, -5.788859859992857, 29.577871835558643],
 [28.038829762188314, -2.0300627046952995, 19.64442545516042],
 [24.874357970743844, -5.814646757253082, 9.979975332350525],
 [13.612786803720851, -5.425849033286961, 7.476824899827078],
 [7.7658499426317915, -4.66591611942822, 6.365145400658089],
 [-3.8988405843327008, -3.8678203005703025, 6.057902364497173],
 [-6, 2, 0]]

#calculate path length and print it
length = environment.calc_path_length(path)
print("the total distance of the path is:", length, "m")

#define the gym environment
env = gym.make('Quadrotor-v0')
#set initial position of the drone to the first position of the path
current_state = env.reset(position=path[0])

#initialise variables to zero and set the time_steps and simulation time
E_tot = 0
tot_err =  0
dt = 0.01
t = 0
sim_time = 130 #130
#print("the average speed of the drone is:", length/sim_time*3.6, "km/h")

#define the controller and initialse reference and drone trajectory
controller = PDcontrolller()
ref_trajectory = {'x':[],'y':[],'z':[]}
drone_trajectory = {'x':[],'y':[],'z':[]}

#Simulate the drone following the global path using the PD controller
while(t<sim_time):
    # --- the desired state ---
    desired_state = environment.rrtstar_path_state(t, path, sim_time)
    
    control_variable = controller.control(desired_state,current_state)
    action = control_variable['cmd_motor_speeds']
    obs, reward, done, info = env.step(action)
    
    #calculating total squared error
    tot_err = tot_err + (obs['x'][0]-desired_state['x'][0])** 2 + (obs['x'][1]-desired_state['x'][1])**2 + (obs['x'][2]-desired_state['x'][2])**2
    
    #calculating total energy
    E_tot = E_tot + sum(np.square(action))

    ref_trajectory['x'].append(desired_state['x'][0])
    ref_trajectory['y'].append(desired_state['x'][1])
    ref_trajectory['z'].append(desired_state['x'][2])
    
    drone_trajectory['x'].append(obs['x'][0])
    drone_trajectory['y'].append(obs['x'][1])
    drone_trajectory['z'].append(obs['x'][2])
    current_state = obs
    t += dt

#plot the final drone_trajectory and the global path in the environment
environment.final_path_plot(path, drone_trajectory)
#make a path suitable for coppelia_sim from the simulated drone trajectory
coppelia_path = list(zip(drone_trajectory['x'],drone_trajectory['y'],drone_trajectory['z']))
#save the coppelia_path to the csv file coppelia_path.csv
file = open('coppelia_path.csv', 'a+', newline ='')
with file:   
    write = csv.writer(file)
    write.writerows(coppelia_path)

# Printing final solutions
print('Total (cumulitive) squared error= ', np.round(tot_err),'\n')
print('Total energy used during flight= ', np.round(E_tot),'\n')

# Below the total time for the trajectory to complete is printed
# this total time is equal to the T variable as defined in the different functions within the paths.py
print('Total time needed to complemete the trajectory path =',sim_time ,'seconds')
print("the average speed of the drone is:", length/sim_time, "m/s")
print("the total distance of the path is:", length, "m")
