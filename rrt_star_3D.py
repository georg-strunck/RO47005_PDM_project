"""
Path planning Sample Code with RRT*
author: Atsushi Sakai(@Atsushi_twi)

adjusted by: Tibbe Lukkassen
A large part of this file is created based on the code from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/RRTStar/rrt_star.py
The code is adjusted for use in 3D and small things are changed for the purpose of this project

This file can be used seperatly to check how it works. It contains a main function which it will use when ran.
"""
import math
import matplotlib.pyplot as plt
from rrt_3D import RRT
import csv

class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y, z):
            super().__init__(x, y, z)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 expand_dis=1,
                 path_resolution=0.001,
                 goal_sample_rate=20,
                 max_iter=200,
                 play_area=None,
                 margin = 0.2,
                 connect_circle_dist=4.0,
                 search_until_max_iter=False,
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        super().__init__(start, goal, obstacle_list, expand_dis,
                         path_resolution, goal_sample_rate,max_iter, play_area, margin)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0],goal[1],goal[2])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    def planning(self):
        """
        rrt star path planning
        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y),new_node.z-near_node.z)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            #if animation:
            #    self.draw_graph(rnd)
            
            self.plot_objects(new_node)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
            

        print("reached max iteration")
        


        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y,n.z) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        r = self.connect_circle_dist
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 + (node.z - new_node.z)**2
                     for node in self.node_list]
        
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(
                edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.z = edge_node.z
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.path_z = edge_node.path_z
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ , _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def main():
    scenario = 2 # was 1
    plt.close('all');
    print("start " + __file__)
    
    if scenario == 1:
        objects_list = [[0.0, 0.0, 0.5, 1.0,1.0,1.0], 
                        [-5.0, -4.0, 0.5, 1.0,1.0,1.0], 
                        [8.0, -2.0, 0.5, 1.0,1.0,1.0], 
                        [15.0, -3.0, 1.5, 3.0,3.0,3.0]]
    

        # Set Initial parameters
        rrt_star = RRTStar(
             start=[0, -2, 0],
             goal=[23, -3, 2],
             play_area = [-8, 25, -5, 2, 0, 4],
             obstacle_list=objects_list,
             expand_dis=1,
             path_resolution=0.001,
             goal_sample_rate=20,
             max_iter=200,
             margin = 0.2,
             connect_circle_dist=4.0,
             search_until_max_iter=False)
        path = rrt_star.planning()
        
        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
        
        rrt_star.final_path_plot(path)
    elif scenario == 2:
        objects_list = [[1.3499996662139893, -0.75, 0.5, 1.0, 1.0, 1.0], 
                        [0.624998927116394, -3.4500019550323486, 0.5, 1.0, 1.0, 1.0], 
                        [5.399999618530273, -3.599998712539673, 0.5, 1.0, 1.0, 1.0], 
                        [7.849998474121094, -0.9999996423721313, 1.5, 3.0, 3.0, 3.0], 
                        [-1.6499996185302734, -4.8499979972839355, 1.0, 4.0, 1.0, 2.0], 
                        [3.97499942779541, 0.9000021815299988, 1.0, 4.0, 1.0, 2.0], 
                        [-4.225001811981201, -1.4500010013580322, 2.0, 1.0, 5.0, 4.0], 
                        [3.6499996185302734, -4.974996089935303, 1.0, 4.0, 1.0, 2.0], 
                        [1.2500001192092896, 3.0000009536743164, 1.0, 4.0, 1.0, 2.0], 
                        [-1.3749998807907104, 5.650001049041748, 1.0, 4.0, 1.0, 2.0], 
                        [-4.700002193450928, 6.299997806549072, 2.0, 1.0, 5.0, 4.0]]
        # Set Initial parameters
        rrt_star = RRTStar(
             start=[7.5, -5, 0],
             goal=[-6, 2.5, 1],
             play_area = [-7, 10, -7, 10, 0, 6],
             obstacle_list=objects_list,
             expand_dis=1.0,
             path_resolution=0.01,
             goal_sample_rate=20,
             max_iter=300,
             margin = 0.2,
             connect_circle_dist=2.0,
             search_until_max_iter=False)
        path = rrt_star.planning()
        
        
        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
            print(type(path)) # nieuw
            rrt_star.final_path_plot(path)
            
            # opening the csv file in 'a+' mode
            file = open('final_path.csv', 'a+', newline ='')
 
            # writing the data into the file
            with file:   
                write = csv.writer(file)
                write.writerows(path)
        
if __name__ == '__main__':
    main()
    