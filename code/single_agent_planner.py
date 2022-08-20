from common_for_search import * 
from mdd import MDD

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, isMDD=False):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
   

    TIME_LIMIT = 0.1
    open_list = []
    paths = []
    optimal_path = None
    optimal_len = -1
    closed_list = dict()
    h_value = h_values[start_loc]
    negative_table = []
    positive_table = []
    if(constraints != None):
        [negative_table, positive_table] = build_constraint_tables(constraints = constraints, agent = agent) 

    #added new key/value pair for time steps   
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step' : 0}   
    push_node(open_list, root)

    #now closed list is indexed by a cell and time step tuple
    closed_list[(root['loc'], root['time_step'])] = root                                      
    

    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        if curr['loc'] == goal_loc:
            if (len(negative_table) == 0):
                final_path = get_path(curr) 
                if (isMDD):
                    mdd = MDD(my_map, start_loc, h_values, len(final_path), positive_table, negative_table)
                    mdd.buildMDD()
                    return final_path, mdd
                else:
                    return final_path
            else:
                max_index = max(negative_table)
                curr_timestep = curr['time_step']

                time_count = max_index
                while curr_timestep < time_count:
                    if is_constrained(goal_loc, goal_loc, time_count, negative_table):
                        break
                    time_count -= 1
                if(curr_timestep >= time_count):
                    final_path = get_path(curr)
                    if(isMDD):
                      
                        
                        mdd = MDD(my_map, start_loc, h_values, len(final_path), positive_table, negative_table)
                        mdd.buildMDD()

                        return final_path, mdd
                    else:
                        return final_path
                    
                
        #expanding current node
        for dir in range(5):
   
       
            child_loc = move(curr['loc'], dir)
        
            #Check if the child_loc position is not out of the maps' bounds
            if (child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0])): 
                continue
           
            #checks if the child_loc in the map is not blocked
            if my_map[child_loc[0]][child_loc[1]]:     
                continue
            
            #generating a child node
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step'] + 1}
            
            #Check if new node satisfies the passed negative constraints, if doesn't -> prune
            if (is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, negative_table)):
                continue
            #Check if new node satisfies the passed positive constraints, if doesn't -> prune 
            if (is_positive_satisfied(parent_node = curr, child_node = child, next_time = curr['time_step'] + 1, positive_constraints = positive_table) == False):
                continue

            
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
    
                    push_node(open_list, child) 
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)
    if(isMDD):
        return None, None
    return None  # Failed to find solutions
