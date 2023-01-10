"""
Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
Based on code by: Atsushi Sakai(@Atsushi_twi)

adjusted by: Tibbe Lukkassen
A large part of this file is created based on the code from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/RRT/rrt.py
The code is adjusted for use in 3D and small things are changed for the purpose of this project

This file can be used seperatly to check how it works. It contains a main function which it will use when ran.
"""
import math
import random
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations

show_animation = True

class RRT:
    """
    Class for RRT planning
    """
    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])
            self.zmin = float(area[4])
            self.zmax = float(area[5])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 expand_dis=1.0,
                 path_resolution=0.05,
                 goal_sample_rate=5,
                 max_iter=300,
                 play_area=None,
                 margin = 0.2,
                 ):
        """
        Setting Parameter
        start:Start Position [x,y,z]
        goal:Goal Position [x,y,z]
        obstacleList:obstacle Positions [[x,y,z,x_size,y_size,z_size],...]
        play_area:stay inside this area [xmin,xmax,ymin,ymax,zmin,zmax]
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.fig = plt.figure()
        self.margin = margin
    
    def plot_objects(self,node):
        plt.clf()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        ax = self.fig.add_subplot(111, projection='3d')
        for i in range(len(self.obstacle_list)):
            x = [self.obstacle_list[i][0]-self.obstacle_list[i][3]/2, self.obstacle_list[i][0]+self.obstacle_list[i][3]/2]
            y = [self.obstacle_list[i][1]-self.obstacle_list[i][4]/2, self.obstacle_list[i][1]+self.obstacle_list[i][4]/2]
            z = [self.obstacle_list[i][2]-self.obstacle_list[i][5]/2, self.obstacle_list[i][2]+self.obstacle_list[i][5]/2]
            for s, e in combinations(np.array(list(product(x, y, z))), 2):
                ax.plot3D(*zip(s, e), color="green")
            
        ax.scatter3D(self.start.x,self.start.y,self.start.z, color ='green')
        ax.scatter3D(self.end.x,self.end.y,self.end.z,  color = 'red')
        ax.scatter3D(node.x,node.y,node.z, "^k")
        
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")
            ax.plot3D([self.play_area.xmin,self.play_area.xmin],[self.play_area.ymin,self.play_area.ymin],[self.play_area.zmin,self.play_area.zmax], color="black")
        
        for node in self.node_list:
            if node.parent:
                #print("after print:", np.array(node.path_x).shape,np.array(node.path_y).shape,np.array(node.path_z).shape)
                ax.plot3D(node.path_x, node.path_y, node.path_z, "-r")
        
        plt.show()
        plt.pause(0.001)
    
    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(rnd_node)
            nearest_node = self.node_list[nearest_ind] 
            
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis) 

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            #if animation and i % 5 == 0:
            #    self.draw_graph(rnd_node)

            
            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y,
                                      self.node_list[-1].z) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            #if animation and i % 5:
            #    self.draw_graph(rnd_node)
            self.plot_objects(rnd_node)

        return None  # cannot find path
    
    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return math.hypot(math.hypot(dx,dy), dz)
    
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([node.x, node.y, node.z])
        return path
    
    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.play_area.xmin, self.play_area.xmax),
                random.uniform(self.play_area.ymin, self.play_area.ymax),
                random.uniform(self.play_area.zmin, self.play_area.zmax))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y,self.end.z)
        return rnd
    
    def get_nearest_node_index(self, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z - rnd_node.z)**2
                 for node in self.node_list]
        minind = dlist.index(min(dlist))
        return minind
    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, theta, alpha = self.calc_distance_and_angle(new_node, to_node)
    
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]
    
        if extend_length > d:
            extend_length = d
    
        n_expand = math.floor(extend_length / self.path_resolution)
    
        for i in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta) * math.cos(alpha)
            new_node.y += self.path_resolution * math.sin(theta) * math.cos(alpha)
            new_node.z += self.path_resolution * math.sin(alpha)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)
    
        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_z.append(to_node.z)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.z = to_node.z
        new_node.parent = from_node
    
        return new_node
    
    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        theta = math.atan2(dy, dx)
        alpha = math.atan2(dz, math.hypot(dx, dy))
        return d, theta, alpha
    
    def check_collision(self, node, obstaclelist):
        if node is None:
            return False
        
        for (ox, oy, oz, dx, dy, dz) in obstaclelist:
            dx_path = [abs(x - ox) for x in node.path_x]
            dy_path = [abs(y - oy) for y in node.path_y]
            dz_path = [abs(z - oz) for z in node.path_z]
            
            for x_dis,y_dis,z_dis in zip(dx_path, dy_path, dz_path):
                if  x_dis >= 0 and x_dis <= dx/2+self.margin and \
                    y_dis >= 0 and y_dis <= dy/2+self.margin and \
                    z_dis >= 0 and z_dis <= dz/2+self.margin:
                    return False
        return True  # safe
    
    def final_path_plot(self, path):
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
                
            ax.scatter3D(self.start.x,self.start.y,self.start.z, color ='green')
            ax.scatter3D(self.end.x,self.end.y,self.end.z,  color = 'red')
            
            if self.play_area is not None:
                plt.plot([self.play_area.xmin, self.play_area.xmax,
                          self.play_area.xmax, self.play_area.xmin,
                          self.play_area.xmin],
                         [self.play_area.ymin, self.play_area.ymin,
                          self.play_area.ymax, self.play_area.ymax,
                          self.play_area.ymin],
                         "-k")
                ax.plot3D([self.play_area.xmin,self.play_area.xmin],[self.play_area.ymin,self.play_area.ymin],[self.play_area.zmin,self.play_area.zmax], color="black")
            
            for coor in range(len(path)-1):
                ax.plot3D([path[coor][0],path[coor+1][0]],[path[coor][1],path[coor+1][1]],[path[coor][2],path[coor+1][2]], color="red")
            plt.show()
        
def main(gx=23.0, gy=-3.0, gz=2.0):
    plt.close('all');
    print("start " + __file__)
   
    objects_list = [[0.0, 0.0, 0.5, 1.0,1.0,1.0], 
                    [-5.0, -4.0, 0.5, 1.0,1.0,1.0], 
                    [8.0, -2.0, 0.5, 1.0,1.0,1.0], 
                    [15.0, -3.0, 1.5, 3.0,3.0,3.0]]
    rrt = RRT(
        start=[0, -2, 0],
        goal=[gx, gy, gz],
        obstacle_list=objects_list,
        play_area=[-8, 25, -5, 2, 0, 4],
        )
    path = rrt.planning()
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
    rrt.final_path_plot(path)
    
if __name__ == '__main__':
    main()