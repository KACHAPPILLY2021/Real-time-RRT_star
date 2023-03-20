import numpy as np
import matplotlib.pyplot as plt
import math
import time
import random

class RT_RRTStar:

    class Node:
        
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent_node = None
            
        def x_movement(self):
            if self.parent_node:
                return [self.x, self.parent_node.x]
            else:
                return []

        def y_movement(self):
            if self.parent_node:
                return [self.y, self.parent_node.y]
            else:
                return []

        def cost_to_come(self):
            if not self.parent_node:
                return 0.0
            x_movement = self.x_movement()
            y_movement = self.y_movement()
            cost = math.hypot(x_movement[1] - x_movement[0],y_movement[1] - y_movement[0])
            node = self.parent_node
            while node.parent_node:
                x_movement = node.x_movement()
                y_movement = node.y_movement()
                cost += math.hypot(x_movement[1] - x_movement[0],y_movement[1] - y_movement[0])
                node = node.parent_node
            return cost

    def __init__(self, start, goal, wall_list, workspace,node_list = []):
        
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.x_lower_limit = workspace[0]
        self.x_upper_limit = workspace[1]
        self.y_lower_limit = workspace[2]
        self.y_upper_limit = workspace[3]
        self.max_extenstion = 0.2
        self.wall_list = wall_list
        self.node_list = node_list
        self.start_time = time.time()
        self.kmax = 100
        self.neighbour_seach_radius = 0.3
        
    def plot_tree(self, random_node=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if random_node is not None:
            plt.plot(random_node.x, random_node.y, "*y")
        for node in self.node_list:
            if node.parent_node:
                plt.plot(node.x_movement(), node.y_movement(), "-g")
        for (sx,sy,ex,ey) in self.wall_list:
            x1, y1 = [sx, ex], [sy, ey]
            plt.plot(x1, y1, "-k")

        plt.plot(self.start.x, self.start.y, "Xb")
        plt.plot(self.end.x, self.end.y, "Hr")
        plt.axis([self.x_lower_limit, self.x_upper_limit, self.y_lower_limit, self.y_upper_limit])
        plt.pause(0.001)
        
    def cost_to_go(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)
    
    def find_path_goal_found(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent_node:
            path.append([node.x, node.y])
            node = node.parent_node
        path.append([node.x, node.y])
        
        return path
    
    def find_path_goal_not_found(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node.parent_node:
            path.append([node.x, node.y])
            node = node.parent_node
        path.append([node.x, node.y])
        
        return path

    @staticmethod
    def is_path_intersecting_with_wall(line_1, line_2):
        delta_x = (line_1[0][0] - line_1[1][0], line_2[0][0] - line_2[1][0])
        delta_y = (line_1[0][1] - line_1[1][1], line_2[0][1] - line_2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(delta_x, delta_y)
        if div == 0:
           return False
        
        d = (det(*line_1), det(*line_2))
        x = det(d, delta_x) / div
        y = det(d, delta_y) / div

        if (x > max(line_1[0][0], line_1[1][0]) or x < min(line_1[0][0], line_1[1][0]) or
            y > max(line_1[0][1], line_1[1][1]) or y <min(line_1[0][1], line_1[1][1]) or
            x > max(line_2[0][0], line_2[1][0]) or x < min(line_2[0][0], line_2[1][0]) or
            y > max(line_2[0][1], line_2[1][1]) or y <min(line_2[0][1], line_2[1][1])):
            return False
        
        return True
    
    def get_random_node(self):
        if random.randint(0, 100) >= 5:
            random_node = self.Node(random.uniform(self.x_lower_limit, self.x_upper_limit),
                            random.uniform(self.y_lower_limit, self.y_upper_limit))
        else:
            random_node = self.Node(self.end.x, self.end.y)
        return random_node
    
    def get_nearest_node_index(self, random_node_node):
        dlist = [math.sqrt((node.x - random_node_node.x) ** 2 + (node.y - random_node_node.y)
                 ** 2) for node in self.node_list]
        minind = dlist.index(min(dlist))

        return minind
    
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    
    def is_obstacle(self, node,obstacle):
        
        if node is None:
            return False
        
        for (sx,sy,ex,ey) in obstacle:
            firstx = node.x_movement()[0]
            secondx = node.x_movement()[1]
            firsty = node.y_movement()[0]
            secondy = node.y_movement()[1]

            A = [firstx, firsty]
            B = [secondx, secondy]

            C = [sx, sy]
            D = [ex, ey]

            if self.is_path_intersecting_with_wall((A, B), (C, D)):
                return False

        return True
    
    def find_next_node(self, from_node, to_node, extend_length=15.0):
    
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        # print(d)
        if extend_length > d:
            extend_length = d
        
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)

        new_node.parent_node = from_node
        
        return new_node

    def path_planning(self):
        
        if not self.node_list:
            self.node_list = [self.start]
        
        start_index = self.get_node_index(self.start.x, self.start.y)
        if self.node_list[start_index].parent_node:
            self.re_root_node(start_index)
            
        i = 0
        
        while time.time() - self.start_time < 0.5:
            
            random_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(random_node)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.find_next_node(nearest_node, random_node, self.max_extenstion)

            if self.is_obstacle(new_node, self.wall_list):
                near_inds = self.find_neighbour(new_node)
                if len(near_inds) < self.kmax:
                    new_node = self.find_best_parents(new_node, near_inds)
                    if new_node and (not self.check_node_in_list(new_node.x, new_node.y)):
                        self.node_list.append(new_node)
                        self.neighbour_rewiring(new_node, near_inds)

            if i % 5 == 0:
                self.plot_tree(random_node)
            i = i + 1
        
        last_index = self.search_best_goal_node()
        if last_index:
            path = self.find_path_goal_found(last_index)
            next_ind = self.get_next_root_index(path)
            return True, path, self.node_list, next_ind

        path = self.find_path_goal_not_found(self.get_node_closest_to_end())
        next_ind = self.get_next_root_index(path)
        return False, path, self.node_list, next_ind

    def find_best_parents(self, new_node, near_inds):
        if not near_inds:
            return None
    
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.find_next_node(near_node, new_node)
            if t_node and self.is_obstacle(t_node,self.wall_list):
                costs.append(t_node.cost_to_come())
            else:
                costs.append(float("inf"))
        min_cost = min(costs)

        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.find_next_node(self.node_list[min_ind], new_node)
        new_node.parent_node = self.node_list[min_ind]
        
        return new_node            

    def search_best_goal_node(self):
        dist_to_goal_list = [self.cost_to_go(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.max_extenstion]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.find_next_node(self.node_list[goal_ind], self.end)
            if self.is_obstacle(t_node, self.wall_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost_to_come() for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost_to_come() == min_cost:
                end_ind = self.get_node_index(self.end.x, self.end.y)
                if end_ind:
                    self.node_list[end_ind].parent_node = self.node_list[i]
                return i

        return None

    def find_neighbour(self, new_node):
        dist_list = [(node.x - new_node.x) ** 2 +
                     (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= self.neighbour_seach_radius  ** 2]
        return near_inds

    def check_node_in_list(self, x, y):
        for node in self.node_list:
            if node.x == x and node.y == y:
                return True
        return False

    def get_node_index(self, x, y):
        for i in range(0,len(self.node_list)):
            node = self.node_list[i]
            if node.x == x and node.y == y:
                return i
        return None
    
    def get_node_closest_to_end(self):
        dist_to_goal_list = [self.cost_to_go(n.x, n.y) for n in self.node_list]
        min_ind = dist_to_goal_list.index(min(dist_to_goal_list))
        return min_ind

    def neighbour_rewiring(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.Node(near_node.x,near_node.y)
            edge_node.parent_node = new_node
            if not edge_node:
                continue
            
            no_collision = self.is_obstacle(edge_node, self.wall_list)
            cost_improved = edge_node.cost_to_come() < near_node.cost_to_come()

            if no_collision and cost_improved:
                self.node_list[i].parent_node = new_node

    def re_root_node(self, next_root_index):
        current_root_ind = self.get_node_index(self.node_list[next_root_index].parent_node.x, self.node_list[next_root_index].parent_node.y)
        self.node_list[next_root_index].parent_node =  None
        self.node_list[current_root_ind].parent_node = self.node_list[next_root_index]
        
    def get_next_root_index(self, path):
        next_ind = 0
        if len(path) > 2:
            next_root_x = path[-2][0]
            next_root_y = path[-2][1]
            next_ind = self.get_node_index(next_root_x,next_root_y)
        elif len(path) == 2:
            next_root_x = path[1][0]
            next_root_y = path[1][1]
            next_ind = self.get_node_index(next_root_x,next_root_y)
        else: 
            next_root_x = path[0][0]
            next_root_y = path[0][1]
            next_ind = self.get_node_index(next_root_x,next_root_y)
        return next_ind
    
def main():
    
    # define start position
    start_node=[-2.0,-0.5]
    # define goal position
    goal_node=[9,-9]
    # gx2 = -gx
    
    # map size
    map = [-10, 10, -10, 10]
    
    # define walls
    obstacle = [(-8,1,-6,1) , (-6,1,-6,8) , (-6,8,-4,8) , (-2,4,6,4) , (-7,-7, -7,-5),(-7,-5,-3,-5) , (-3, -5 ,-3,-2),(2,-5 , 8,-5)]
    
    # set timer
    starttime = time.time()
    goal = goal_node
    # duration of simulation
    t = 120.0
    
    # list to store all the generated node
    node_list = []

    # create and object of the class RT_RRT
    RRTstar = RT_RRTStar(start=start_node,goal=goal_node,workspace=map,wall_list=obstacle,node_list = [] )

    # find the path to reach the goal
    is_path_found, path, node_list, next_ind = RRTstar.path_planning()
    
    while time.time() - starttime < t:
        
        if (time.time() - starttime) < 80:
            goal=[goal_node[0], goal_node[1]+ (time.time() - starttime)/8]
        
        RRTstar = RT_RRTStar(start=[node_list[next_ind].x, node_list[next_ind].y],
                      goal=goal,
                      workspace=[-10, 10, -10, 10],
                      wall_list=obstacle,
                      node_list = node_list)
        
        is_path_found, path, node_list, next_ind = RRTstar.path_planning()

    if is_path_found:
        if path:        
            RRTstar.plot_tree()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.pause(0.1)
            plt.show()
    else:
        print("path not found")
        
    endtime = time.time()
    print(endtime - starttime)
        
if __name__ == '__main__':
    main()