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
  [60.0123720001508, -119.92776997158217, 0.12743416877885755],
  [59.844397396140856, -113.26271340216552, 11.393357646585908],
  [41.91049414309742, -90.03927322124242, 50.933986904310046],
  [37.62405658809566, -82.77688892638396, 58.323578903522446],
  [31.28386964288192, -46.08210466845834, 75.43466078587905],
  [28.835528338388468, -25.517003401947076, 72.03312032281296],
  [22.102081888368037, -18.98447164292926, 52.43364057081649],
  [9.832018114757643, -6.0459272122380145, 22.020165209500444],
  [-6, 2, 0]]

#Smoothen the path to a max sharp angle of 1
path = environment.remove_sharp_angles(path, 1)
#calculate path length and print it
length = environment.calc_path_length(path)

#define the gym environment
env = gym.make('Quadrotor-v0')
#set initial position of the drone to the first position of the path
current_state = env.reset(position=path[0])

#initialise variables to zero and set the time_steps and simulation time
E_tot = 0
tot_err =  0
dt = 0.01
t = 0

#Set the average speed in km/h
v_avg = 12 #km/h

sim_time = length/(v_avg/3.6)

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
print("the total distance of the path is:", length, "m")
print("the average speed of the drone is:", length/sim_time*3.6, "km/h")

print('Total (cumulitive) squared error= ', np.round(tot_err),'\n')
print('Total energy used during flight= ', np.round(E_tot),'\n')

# Below the total time for the trajectory to complete is printed
# this total time is equal to the T variable as defined in the different functions within the paths.py
print('Total time needed to complemete the trajectory path =',sim_time ,'seconds')