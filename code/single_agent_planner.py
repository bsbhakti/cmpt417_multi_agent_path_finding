from calendar import c
import heapq
from operator import truediv

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def put_in_table(constraint_table, timestep, loc,type,mainType):
    if(timestep in constraint_table[mainType][type]):
        constraint_table[mainType][type][timestep].append(loc)
    else:
        constraint_table[mainType][type][timestep] = [loc]
    return constraint_table


def build_constraint_table(constraints, agent, isCbs):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
   
    # dict indexed by timestep. each index has an array of locations the agent cannt be at at that timestep
    # {'a1': 2, 'a2': 1, 'loc': [(1, 5)], 'timestep': 2, 'vertex': True, 'positive':True} //const looks like this
    constraint_table = {"negative":{'edge': {}, 'vertex': {}, 'end': {}} , "positive": {'edge': {}, 'vertex': {} }}
    main_type = "negative"
    for constraint in constraints:
        # print("this is constraint",constraint)
        if(isCbs):
            if constraint['agent'] != agent:
                continue

        if(constraint["positive"] == False):
            main_type = "negative"
            # print("here")
        else:
            # print("here")
            main_type = "positive"

        if(constraint['end'] == True):
            constraint_table = put_in_table(constraint_table, constraint['timestep'], constraint['loc'][0],'end', main_type)

            # constraint_table['end'][constraint['timestep'] ]
        else:
            if(len(constraint['loc']) == 1):
                constraint_table = put_in_table(constraint_table, constraint['timestep'], constraint['loc'][0],'vertex',main_type)
            
            else:
                constraint_table = put_in_table(constraint_table, constraint['timestep'], constraint['loc'],'edge',main_type)


    # d = {}
    # d[constraints["loc"]] = agent
    # constraint_table[constraints["timestep"]]= d
    # print("TABLE")
    # print(constraint_table)
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    #check pos vertex constraint
    if(next_time in constraint_table["positive"]['vertex']):
        if (next_loc not in constraint_table["positive"]['vertex'][next_time]):
            # print("hew")
            return True

    #check pos edge constraint
    if(next_time in constraint_table["positive"]['edge']):
        # print("checking edge", (curr_loc, next_loc),next_time )
        if ( [curr_loc, next_loc] not in constraint_table["positive"]['edge'][next_time] ):
            # print("returning True", (curr_loc, next_loc),next_time )
            # print("hew1")
            return True

    #check vertex constraint
    if(next_time in constraint_table["negative"]['vertex']):
        if (next_loc in constraint_table["negative"]['vertex'][next_time]):
            # print("hew")
            return True

    #check edge constraint
    if(next_time in constraint_table["negative"]['edge']):
        # print("checki÷ng edge", (curr_loc, next_loc),next_time )
        if ( [curr_loc, next_loc] in constraint_table["negative"]['edge'][next_time] ):
            # print("returning True", (curr_loc, next_loc),next_time )
            print("hew1")
            return True

    for time in constraint_table["negative"]['end'].keys():
        if(time <= next_time and constraint_table["negative"]['end'][time] == [next_loc]):
            print("he2")

            return True

    # print("fa")
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, upperbound, isCbs):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    # print("a star is getting this ", constraints)
    # return

    constraint_table = build_constraint_table(constraints, agent, isCbs)
    print("this is constraint table", constraint_table) 

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time'])] = root
    goal_found = False
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if(curr["time"] <= upperbound):
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        ## maybe change this
            if curr['loc'] == goal_loc:
                goal_found = True
                for i in constraint_table["negative"]['vertex'].keys():
                    if i > curr['time']:
                        if curr['loc'] in constraint_table["negative"]['vertex'][i]:
                            goal_found = False
                            # p = get_path(curr)
                            # if(get_sum_of_cost(p) == 26):
                            #     print("making goal False",p )
                            # break
                if(goal_found):
                    print("found")
                    return get_path(curr)
        
            for dir in range(5):
                child_loc = move(curr['loc'], dir)
                # if my_map[child_loc[0]][child_loc[1]]:
                #     continue
                if child_loc not in h_values:
                # not valid_state(my_map,child_loc):
                    continue

                # print(curr)
                child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': h_values[child_loc],
                        'parent': curr,
                        'time': curr['time'] + 1 }
                # mayve we can prune when we pop it, rn pruning is done at node generation? 
                # print("sa")
                if(is_constrained(curr['loc'], child['loc'], child['time'],constraint_table )):
                    print("It is constrained for loc", child['loc'], "at time", child['time'])
                    continue
                if ((child['loc'], child['time'])) in closed_list:
                    existing_node = closed_list[(child['loc'], child['time'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['time'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['time'])] = child
                    push_node(open_list, child)
                # print("here looping")

    return None  # Failed to find solutions
