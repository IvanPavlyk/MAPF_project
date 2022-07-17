import heapq
from locale import currency

from numpy import positive

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
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


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraint_table = dict()
    positive_constraint_table = dict()
    # print("constrs", constraints)
    for constraint in constraints:
        
        #constructing negative constraints
        
        #Task 4.1 added code to handle positive constraints
        if ("positive" not in constraint.keys()):
            constraint['positive'] = False
        if(constraint['agent'] == agent):
            
            if(constraint['positive'] == False):
            
                if (constraint['time_step'] in constraint_table):                                   #if table constains constraints in the time_step, append to it
                    constraint_table[constraint['time_step']].append(constraint['loc'])
                else:
                    constraint_table[constraint['time_step']] = [constraint['loc']]                 #if not, create a new time_step/location key/value pair
            else:
                
                if (constraint['time_step'] in positive_constraint_table):                          #if table constains constraints in the time_step, append to it
                    positive_constraint_table[constraint['time_step']].append(constraint['loc'])
                else:
                    positive_constraint_table[constraint['time_step']] = [constraint['loc']]        #if not, create a new time_step/location key/value pair
    return [constraint_table, positive_constraint_table]


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
    
    if (next_time in constraint_table):                 #if next_time key exists in the constraint_table
        constrained_cells = constraint_table[next_time]  #obtained constrained cells/edges at the given time step
        
        for constrain in constrained_cells:
            #check vertex constraint 
            if ( len(constrain) == 1):
                if(constrain == [next_loc]):
                    return True
            #check edge constraint
            else: 
                if([curr_loc, next_loc] == constrain):        
                    return True
    return False
   
    


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


#Checks if the constraint at the child's node time_step satisfies the list of positive constraints 
def is_positive_satisfied(parent_node,child_node , positive_constraints):
   
    if(len(positive_constraints) != 0):
       
        child_time_step = child_node['time_step']
        # print('child time step', child_time_step)
        parent_loc = parent_node['loc']
        child_loc = child_node['loc']
        if(child_time_step in positive_constraints):
            constraint = positive_constraints[child_time_step]

            
            if(len(constraint[0]) == 1):
                if(constraint[0] != [child_loc]):
                    return False
            else:
            
                if(constraint[0][1] != parent_loc or constraint[0][0] != child_loc): 
                    return False

    return True 

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    # print("Finding path for agent: ", agent)
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    
    #Task 1.2, create constraint table beforeurr['loc'], child_loc, curr['time_step'] + 1,constraint_table generating root node
    [constraint_table, positive_table] = build_constraint_table(constraints = constraints, agent = agent) 
   
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step' : 0}    #added new key/value pair for time steps
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root                                      #now closed list is indexed by a cell and time step tuple
    



    #Task 1.4, find earliest goal time step from constraints table
    if(len(constraint_table) != 0):
        earliest_goal_timestep = (max(constraint_table.keys()))
    #if doesn't constraint table for agent is empty, then the earliest is 0
    
    while len(open_list) > 0:
        curr = pop_node(open_list)
       
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['time_step'] >= earliest_goal_timestep:
            return get_path(curr)
       
                
        #expanding current node
        for dir in range(4):
            
            child_loc = move(curr['loc'], dir)
            
            #Check if the child_loc position is not out of the maps' bounds
            if (child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0])): 
                continue
            
            #checks if the child_loc in the map is not blocked
            if my_map[child_loc[0]][child_loc[1]]:     
                continue
            
            #Task 1.2, check if new node satisfies the passed  constraints, if doesn't -> prune
            if (is_constrained(curr['loc'], child_loc, curr['time_step'] + 1,constraint_table)):
                continue
            
            #generating a child node
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step'] + 1}
            if (is_positive_satisfied(curr, child, positive_table) == False):
                    continue
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
    
                    push_node(open_list, child)
                  
            else:

                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
