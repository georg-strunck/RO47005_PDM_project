import gym
#import env
import numpy as np 
#from paths import circle_traject, hover, diamond, tud, rrtstar_path
from paths import rrtstar_path
from controller import PDcontrolller
from matplotlib import pyplot as plt
import csv

env = gym.make('Quadrotor-v0')

# --- current_state to be adjusted per trajectory ---

# with open('final_path.csv') as f:
#         reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
#         data = list(reader)
#         #print(data[1:])
#         data = data[1:]

current_state = env.reset(position=[0,0,0])
#current_state = env.reset(position=data[0])

E_tot = 0
tot_err =  0
dt = 0.01
t = 0

controller = PDcontrolller()
ref_trajectory = {'x':[],'y':[],'z':[]}
drone_trajectory = {'x':[],'y':[],'z':[]}
while(t<30):
    # --- the desired state ---
    desired_state = rrtstar_path(t)
    
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

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(ref_trajectory['x'],ref_trajectory['y'],ref_trajectory['z'], 'blue')
ax.plot3D(drone_trajectory['x'],drone_trajectory['y'],drone_trajectory['z'], 'red')
plt.show()

# Printing final solutions
print('Total (cumulitive) squared error= ', np.round(tot_err),'\n')
print('Total energy used during flight= ', np.round(E_tot),'\n')
# Below the total time for the trajectory to complete is printed
# this total time is equal to the T variable as defined in the different functions within the paths.py
print('Total time needed to complemete the trajectory path = 12 seconds')