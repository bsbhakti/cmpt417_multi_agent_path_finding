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
    if len(path1) > len(path2):
       longer_list = path1
       shorter_list = path2
       agent1 = i
       agent2 = j
    else:
        longer_list = path2
        shorter_list = path1
        agent1 = j
        agent2 = i

    for time, pos in enumerate(longer_list):
        #vertex collision
        if(get_location(shorter_list,time) == pos):
            # print(get_location(path2,time), pos)
            return {'a1': agent1, 'a2': agent2, 'loc': [pos], 'timestep':time, 'vertex': True}
        #edge collision
        if(get_location(shorter_list,time+1) == pos and get_location(shorter_list,time) == get_location(longer_list,time+1)):
            # print("found edge collision", pos,get_location(path2,time+1), get_location(path1,time-1), time, i )
            return {'a1': agent1, 'a2': agent2, 'loc': [pos,get_location(longer_list,time+1)], 'timestep':time+1, 'vertex': False}
      

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
            collision = detect_collision(p1,p2,i,i+j+1)
            if(collision is not None):
                res.append(collision)
    # print("these are first collisions",res)
    return res


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst



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
    if(collision is None):
        return None

    if(collision['vertex']):
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False, "positive": False}
        cons2 = {"agent": collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False,  "positive": False}
        res.append(cons1)
        res.append(cons2)

    else:
        # {'a1': i, 'a2': j, 'loc': [pos,get_location(path2,time+1)], 'timestamp':time+1, 'vertex': False}
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": False}
        cons2 = {"agent": collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": False}
        res.append(cons1)
        res.append(cons2)

    # print("after standard splitting", res)
    return res

# def paths_violate_constraint(paths, constraint):
#     ret = []
#     for i in paths:
#         if(i == constraint["agent"]):
#             continue
#         if(constraint["vertex"]): #if vertex cons
#             if(paths[constraint["timestep"]] == constraint["loc"][0]):
#                 ret.append(i)
#         else:
#             if(paths[constraint["timestep"] -1] == constraint["loc"][0] 
#             and paths[constraint["timestep"] ] ==constraint["loc"][1]  ):
#                 ret.append(i)

#     return ret

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
    res = []
    # print(collision)
    if(collision is None):
        return None

    randomAgent = random.randint(0,1)
    if(randomAgent):
        posAgent = collision['a1']
    else:
        posAgent = collision['a2']

    if(collision['vertex']):
        cons1 = {"agent": posAgent, 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False, "positive": True}
        cons2 = {"agent": posAgent, 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False,  "positive": False}
        res.append(cons1)
        res.append(cons2)

    else:
        # {'a1': i, 'a2': j, 'loc': [pos,get_location(path2,time+1)], 'timestamp':time+1, 'vertex': False}
        cons1 = {"agent": posAgent, 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": True}
        cons2 = {"agent": posAgent, 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": False}
        res.append(cons1)
        res.append(cons2)
    return res
    
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
        # print("these are heuristics", self.heuristics)

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
        # upperbound = len(self.my_map) * len(self.my_map[0])
        upperbound = float('inf')

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        max_path_length = 0
        for i in range(self.num_of_agents):  # Find initial path for each agent
            # print("i am calling a_star", i)
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'],upperbound, True)
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            # print("path", path)
            curr_path_length = len(path)
            max_path_length = max(max_path_length, curr_path_length)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        # return
        self.push_node(root)
        # print("these are the collisions ", root["collisions"])
        # print("these are the paths ", root["paths"])


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
        while(len(self.open_list) >0 ):
            node = self.pop_node()
            # print(f"Popped node has {len(node['collisions'])} collisions and length {node['cost']}")
            if(node["collisions"] == []):
                print(f"Popped node has length {node['cost']}")
                print("Total expanded ",self.num_of_expanded)
                return node["paths"]
            collision = node["collisions"][0]
            # print(collision)
            # break
            if(disjoint):
                constraints = disjoint_splitting(collision)
            else:
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
                # upperbound = ( len(self.my_map) * len(self.my_map[0])) + node["cost"]
                upperbound = float('inf')
                
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], 
                agent, newNode["constraints"], upperbound, True)
                # print("old path", newNode["paths"])
                # print("new path found for agent",agent,path, len(path), upperbound)
                # break
                if(path is not None):
                    newNode["paths"][agent] = path
                    newNode["collisions"] = detect_collisions(newNode["paths"])
                    newNode["cost"] = get_sum_of_cost(newNode["paths"])

                    if disjoint and constraint["positive"]:
                        # print("Going in pos constraint loop, doing a star again")
                        recompute_agents = paths_violate_constraint(constraint,newNode["paths"])
                        # print("Recomputing:", len(recompute_agents))
                        newPathFound = 0

                        for i in recompute_agents:
                            modifiedConstraint = deepcopy(constraint)
                            constraint["agent"] = i
                            constraint["positive"] = False
                            newNode['constraints'].append(modifiedConstraint)
                            newPath = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, newNode['constraints'], upperbound, True)
                            if newPath is None:
                                print("Path not found")
                                continue 
                            newPathFound+=1
                            # print("this is the new path found for agent ", i, newPath) 
                            # print("oldPath:",  newNode['paths'][i]) 

                            newNode['paths'][i] = newPath
                            newNode['collisions'] = detect_collisions(newNode['paths'])
                            newNode['cost'] = get_sum_of_cost(newNode['paths'])

                        if newPathFound == recompute_agents:
                            # print("pushing node disjoint")
                            self.push_node(newNode)
                    else:
                        # print("pushing node")
                        self.push_node(newNode)

                    # print("these are the collisions", newNode["coll"])
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
