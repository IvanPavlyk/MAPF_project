from dis import dis
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, pop_node, push_node


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    path1_len = len(path1)
    path2_len = len(path2)
   
    max_time_steps = max(path1_len, path2_len)
    for time_step in range(max_time_steps):
        agent1_loc_curr = get_location(path1,time_step)
        agent2_loc_curr = get_location(path2, time_step)
        agent1_loc_prev = get_location(path1,time_step + 1)
        agent2_loc_prev = get_location(path2,time_step + 1)

        #detect vertex collision
        if (agent1_loc_curr == agent2_loc_curr):
            # print("Vertex conflict: ", {'loc': [(agent1_loc_curr)], 'time_step' : time_step})
            return {'loc': [agent1_loc_curr], 'time_step' : time_step}
        
        #detect edge collision
        if (agent1_loc_curr == agent2_loc_prev and agent1_loc_prev == agent2_loc_curr ):
            # print("Edge conflict", {'loc': [(agent2_loc_curr), (agent2_loc_prev)], 'time_step' : time_step})
            return {'loc': [ agent2_loc_curr, agent2_loc_prev], 'time_step' : time_step + 1}
    
    return None  
     


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    agents_num = len(paths)
    collisions = list()

    for agent1_id in range(0,agents_num-1):
        for agent2_id in range (agent1_id+1,agents_num):
            detected_collision = detect_collision(paths[agent1_id], paths[agent2_id])
            if ((detected_collision) != None):
                collisions.append({'a1' : agent1_id, 'a2' : agent2_id, 'loc' : detected_collision['loc'], 'time_step' : detected_collision['time_step']})
    return collisions



def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    agent1_id = collision['a1']
    agent2_id = collision['a2']
    collision_loc = collision['loc']
    time_step = collision['time_step']
    
    #if vertex collision 
    if(len(collision_loc) == 1):
        return [{'agent' : agent1_id, 'loc': collision_loc, 'time_step': time_step},
                {'agent' : agent2_id, 'loc': collision_loc, 'time_step': time_step}]
    
    #if edge collision 
    else:
        return [{'agent' : agent1_id, 'loc': [collision_loc[1], collision_loc[0]], 'time_step': time_step},
                {'agent' : agent2_id, 'loc': collision_loc, 'time_step': time_step}]

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
    
    agents =[collision['a1'], collision['a2']]
    collision_loc = collision['loc']
    time_step = collision['time_step']
    # chosen_agent = random.randint(0,1) #choose one of the colliding agents randomly 
    chosen_agent = 0 #choose one of the colliding agents randomly 

    agent = agents[chosen_agent]
    
    #if vertex collision 
    if(len(collision_loc) == 1):
        return [{'positive': True,'agent' : agent, 'loc': collision_loc, 'time_step': time_step},
                {'positive': False,'agent' : agent, 'loc': collision_loc, 'time_step': time_step}]
    
    #if edge collision 
    else:
        
        loc = []
        if (chosen_agent == 0):
            loc = [collision_loc[1], collision_loc[0]]
        else:
            loc = collision_loc
        return [{'positive': True, 'agent' : agent, 'loc': loc, 'time_step': time_step},
                {'positive': False, 'agent' : agent, 'loc': loc, 'time_step': time_step}]

#Task 4.3 
#Gives a list of agent ids that violate passed positive constraint from given list of paths
def paths_violate_constraint(paths, constraint):
    
    # if (constraint['positive']): 
    #     violated_agentsIds = list()
    #     agents_num = len(paths) 
        
    #     for agent in range (agents_num): 
    #         if (agent != constraint['agent']):
    #             #build a pseudo path with the passed constrain 
    #             temp_path = [None] * (constraint['time_step'] + 1)
   
    #             if (len(constraint['loc']) == 1):
    #                 temp_path[constraint['time_step']] = constraint['loc'][0]
    #             else:
    #                 temp_path[constraint['time_step'] - 1] = constraint['loc'][0]
    #                 temp_path[constraint['time_step']] = constraint['loc'][1]
                
    #             #detect collisions with the pseudo constructed path
    #             collision = detect_collision(paths[agent], temp_path)
    #             if(collision is not None):
    #                 violated_agentsIds.append(agent)
    #     print("paths ", paths)
    #     print("Constraint ", constraint)
    #     print("list of agents ", violated_agentsIds)
    #     return violated_agentsIds
    # return []
    
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['time_step'])
        prev = get_location(paths[i], constraint['time_step'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    # print("Paths ", paths)
    # print("Constrasint = ", constraint)
    # print("list of agents ", rst)
    return rst

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
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
         
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
       

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while (len(self.open_list) > 0):
            
            node = self.pop_node()
            if (len(node['collisions']) == 0): #if no collisions return paths    
                print("PAths", node['paths'])
                print("Sum of costs", node['cost'])
                return node['paths']
            
            
            collision = node['collisions'][0] #take one collision
         
            constraints = disjoint_splitting(collision)
            # print("constraints", constraints)
        
            for constraint in constraints:
                #create an empty node Q
                Q = {
                    'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []
                    }
              
              
                #Copy all constraints from the parent node and add additional constraint 
                Q['constraints'] = node['constraints'].copy()           #deep copy 
                Q['constraints'].append(constraint)                    
                Q['paths'] = node['paths'].copy()
               
                ai = constraint['agent']
                
                #build new path with the new constraint 
                path = a_star(  my_map = self.my_map,
                                start_loc = self.starts[ai], 
                                goal_loc = self.goals[ai], 
                                h_values = self.heuristics[ai],
                                agent = ai,
                                constraints = Q['constraints'])
                
                if (path != None):
                    # print("Path is not none")
                    #replace the specified agent's path with the new (containing new constrain)
                    Q['paths'] [ai] = path

                
                    #check whether paths for other agents have been violated
                    broken_agents_paths  = []
                    #if constraint is positive then check if this constraint violates other agents' plans 
                    if(constraint['positive'] == True):
                        broken_agents_paths = paths_violate_constraint(Q['paths'], constraint)    
                    is_path_found = True
                    
                    if(len(broken_agents_paths) != 0):
                        
                        for agent in broken_agents_paths:
                            #build negative constrain that prohibits the broken agent to violate the positive constraint
                            constraint_new = {  'positive': False,
                                                'agent': agent, 
                                                'loc': constraint['loc'],
                                                'time_step': constraint['time_step']}
                            # print("Interating constraint ", constraint['loc'])
                            # print("constraint = ", constraint_new['loc'])
                            Q['constraints'].append(constraint_new)
                            
                            #generate new path with new negative constraint
                            updated_path = a_star   (my_map = self.my_map,
                                                    start_loc = self.starts[agent], 
                                                    goal_loc = self.goals[agent], 
                                                    h_values = self.heuristics[agent],
                                                    agent = agent,
                                                    constraints = Q['constraints'])
                            #break if updated path doesn't exist
                            if (updated_path == None):
                                is_path_found = False
                                break
                            #else add the path to high-level node
                            Q['paths'][agent] = updated_path
                    
                    if(is_path_found == False):
                        continue        

                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)
                else:
                    print("Constraint", constraint)
                    print("Q constraints", Q['constraints'])
                    print("Start",  self.starts[ai])
                    print("Goal ", self.goals[ai])
                    print("path is non")
                
            
        
        raise BaseException("No solutions")
       

           
            
    
    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
