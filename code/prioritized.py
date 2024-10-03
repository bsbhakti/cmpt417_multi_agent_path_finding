import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        # constraints_0 = [{'agent': 0, 'loc': [(1,5)], 'timestep': 10},{'agent': 0, 'loc': [(1,5)], 'timestep': 4}]
        # constraints_1 = [{'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1}]
        # constraints_1 = [{'agent': 1, 'loc': [(1,4)], 'timestep': 2},{'agent': 1, 'loc': [(1,3)], 'timestep': 2},
        # {'agent': 1, 'loc': [(1,2)], 'timestep': 2}] # for Task 1.5

        all_constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            if(i == 0):
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, all_constraints)
            else: 
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, all_constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################
            # at the end of the path, the agent is permanently at the goal so for all time stamps >= len(path)
            # the goal is occupied
            for index,position in enumerate(path[1:]):
                all_constraints.append({'loc':[position], 'timestep': index+1, 'end':False })
                # print("adding cons ",[position, path[index]], index+1)
                all_constraints.append({'loc':[position, path[index]], 'timestep': index+1, 'end':False })
            all_constraints.append({'loc':[path[-1]], 'timestep': len(path), 'end':True })

            # print("th is is all", all_constraints)

                



        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
