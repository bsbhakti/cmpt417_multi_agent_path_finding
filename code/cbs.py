import time as timer
import heapq
import random
from unittest.mock import Base
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from copy import deepcopy


def detect_collision(path1, path2, i,j):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for time, pos in enumerate(path1):
        #vertex collision
        if(get_location(path2,time) == pos):
            # print(get_location(path2,time), pos)
            return {'a1': i, 'a2': j, 'loc': [pos], 'timestep':time, 'vertex': True}
        #edge collision
        if(get_location(path2,time+1) == pos and get_location(path2,time) == get_location(path1,time+1)):
            # print("found edge collision", pos,get_location(path2,time+1), get_location(path1,time-1), time, i )
            return {'a1': i, 'a2': j, 'loc': [pos,get_location(path1,time+1)], 'timestep':time+1, 'vertex': False}

def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    res = []

    for i, p1 in enumerate(paths):
        for j, p2 in enumerate(paths[i+1:]):
            # print()
            res.append(detect_collision(p1,p2,i,i+j+1))
    # print("these are first collisions",res)
    return res

def equalize_path_lengths(paths, max_path_length):
    for path in paths:
        curr_length = len(path)
        if(curr_length < max_path_length):
            last_pos = path[-1]
            for _ in range(curr_length, max_path_length):
                path.append(last_pos) 
        # print("after eq",path)



def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    res = []
    # print(collision)
    if(collision['vertex']):
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False}
        cons2 = {"agent": collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False}
        res.append(cons1)
        res.append(cons2)

    else:
        # {'a1': i, 'a2': j, 'loc': [pos,get_location(path2,time+1)], 'timestamp':time+1, 'vertex': False}
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":False,"end":False}
        cons2 = {"agent": collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'],"vertex":False,"end":False}
        res.append(cons1)
        res.append(cons2)

    # print("after standard splitting", res)
    return res

def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        upperbound = len(self.my_map) * len(self.my_map[0])
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        max_path_length = 0
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'],upperbound)
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            # print("path", path)
            curr_path_length = len(path)
            max_path_length = max(max_path_length, curr_path_length)

        root['cost'] = get_sum_of_cost(root['paths'])
        # make all paths length equal to the longest path
        equalize_path_lengths(root["paths"], max_path_length)
        root['collisions'] = detect_collisions(root['paths'])
        # return
        self.push_node(root)

        # # Task 3.1: Testing
        # print(root['collisions'])

        # # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        # while(len(self.open_list) >0 and self.num_of_expanded <=6):
        while(len(self.open_list) >0):
            node = self.pop_node()
            if(node["collisions"] == [None]):
                return node["paths"]
            collision = node["collisions"][0]
            # print(collision)
            # break
            constraints = standard_splitting(collision)
            # print("these are the constraints", constraints)
            for constraint in constraints:
                print("Solving this cons", constraint)
                # print("before",node["constraints"])
                newNode = {'cost': 0,
                'constraints': deepcopy(node["constraints"]), 
                'paths': deepcopy(node["paths"]),
                'collisions': []}
                newNode["constraints"].append(constraint)

                # print("here", node["constraints"])
                # print("after", newNode["constraints"])
                # break

                agent = constraint["agent"]
                # print("agent", agent)
                # break
                upperbound = ( len(self.my_map) * len(self.my_map[0])) + node["cost"]
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], 
                agent, newNode["constraints"], upperbound)
                # print("old path", newNode["paths"])
                # print("new path found for agent",agent,path, len(path), upperbound)
                # break
                if(path is not None):
                    newNode["paths"][agent] = path
                    newNode["collisions"] = detect_collisions(newNode["paths"])
                    newNode["cost"] = get_sum_of_cost(newNode["paths"])
                    self.push_node(newNode)
                # break


        print("NO SOLUTIONS")
        raise BaseException("No solution")
        # self.print_results(root)
        # return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
