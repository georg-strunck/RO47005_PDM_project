import gym
import numpy as np 
from paths import rrtstar_path, find_path_rrt_star, final_path_plot, load_object_list, play_area_from_objects
from controller import PDcontrolller

env = gym.make('Quadrotor-v0')
        
objects_list = load_object_list('bboxes_objects.csv','center_positions_objects.csv')
play_area = play_area_from_objects(objects_list)
start = [-6, 2, 0]
end = [60, -120, 0]

path = find_path_rrt_star(objects_list, start, end, play_area)

current_state = env.reset(position=path[0])

E_tot = 0
tot_err =  0
dt = 0.01
t = 0

controller = PDcontrolller()
ref_trajectory = {'x':[],'y':[],'z':[]}
drone_trajectory = {'x':[],'y':[],'z':[]}
while(t<150):
    # --- the desired state ---
    desired_state = rrtstar_path(t, path)
    
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

final_path_plot(path, objects_list, play_area, drone_trajectory, start, end)


# Printing final solutions
print('Total (cumulitive) squared error= ', np.round(tot_err),'\n')
print('Total energy used during flight= ', np.round(E_tot),'\n')

# Below the total time for the trajectory to complete is printed
# this total time is equal to the T variable as defined in the different functions within the paths.py
print('Total time needed to complemete the trajectory path = 100 seconds')
