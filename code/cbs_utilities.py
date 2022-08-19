import random
from common_for_search import get_location

def detect_collision(path1, path2):
    ##############################
    # Return the first collision that occurs between two robot paths (or None if there is no collision)
    # There are two types of collisions: vertex collision and edge collision.
    # A vertex collision occurs if both robots occupy the same location at the same timestep
    # An edge collision occurs if the robots swap their location at the same timestep.
    
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
            return {'loc': [agent1_loc_curr], 'time_step' : time_step}
        
        #detect edge collision
        if (agent1_loc_curr == agent2_loc_prev and agent1_loc_prev == agent2_loc_curr ):
            return {'loc': [ agent2_loc_curr, agent2_loc_prev], 'time_step' : time_step + 1}
    return None  
     

def detect_collisions(paths):
    ##############################
    # Return a list of first collisions between all robot pairs.
    # A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    # causing the collision, and the timestep at which the collision occurred.
 
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
    # Return a list of (two) constraints to resolve the given collision
    # Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                  specified timestep, and the second constraint prevents the second agent to be at the
    #                  specified location at the specified timestep.
    # Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                specified timestep, and the second constraint prevents the second agent to traverse the
    #                specified edge at the specified timestep

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
    # Return a list of (two) constraints to resolve the given collision
    # Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                  specified timestep, and the second constraint prevents the same agent to be at the
    #                  same location at the timestep.
    # Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                specified timestep, and the second constraint prevents the same agent to traverse the
    #                specified edge at the specified timestep
    # Choose the agent randomly
    
    agents =[collision['a1'], collision['a2']]
    collision_loc = collision['loc']
    time_step = collision['time_step']
    chosen_agent = random.randint(0,1) #choose one of the colliding agents randomly 
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
   

#Gives a list of agent ids that violate passed positive constraint from given list of paths
def paths_violate_constraint(paths, constraint):
    
    if (constraint['positive']): 
        violated_agentsIds = list()
        agents_num = len(paths) 
        
        for agent in range (agents_num): 
            if (agent != constraint['agent']):
                #build a pseudo path with the passed constrain 
                temp_path = [None] * (constraint['time_step'] + 1)
   
                if (len(constraint['loc']) == 1):
                    temp_path[constraint['time_step']] = constraint['loc'][0]
                else:
                    temp_path[constraint['time_step'] - 1] = constraint['loc'][0]
                    temp_path[constraint['time_step']] = constraint['loc'][1]
                
                #detect collisions with the pseudo constructed path
                collision = detect_collision(paths[agent], temp_path)
                if(collision is not None):
                    violated_agentsIds.append(agent)
        return violated_agentsIds

    return []


def doesContainCardinalConflict(mdd1, mdd2):
        levels1 = mdd1.levels
        levels2 = mdd2.levels
        min_length = min(len(levels1), len(levels2))
        
        for i in range (min_length):
            list1 = levels1[i]
            list2 = levels2[i]     
            if (len(list1) == 1 and len(list2) == 1):
                # print("list1=",list1[0].location)
                # print("list2=",list2[0].location)
                if(list1[0].location == list2[0].location):
                    # print("found cardinal conflict")
                    # print("list1=", list1)
                    # print("list2=",list2)
                    return True 
        return False

def areAgentsDependent(mdd1, mdd2):
        ret = True
        
        levels1 = mdd1.levels
        levels2 = mdd2.levels

        min_level = levels1 if len(levels1) <= len(levels2) else levels2 
        min_level_length =len(min_level)

        merged_levels = [[] for _ in range (min_level_length)]

        for level in range(min_level_length):
            list1 = levels1[level]
            list2 = levels2[level]

            #for each location do a cartisian product
            for loc_1 in range(len(list1)):
                for loc_2 in range(len(list2)):
                    if(list1[loc_1].location == list2[loc_2].location):
                        break
                    else: 
                        merged_levels[level].append([list1[loc_1], list2[loc_2]])
        
     
        
        for pairs in merged_levels[-1]: #last level of merged_levels
           for cell in pairs:
                if (min_level[-1][0].location == cell): 
                    ret = False

        return ret


def getBetterCollision(collisions, mdds): 
        for collision in collisions:
            agent1_id = collision['a1']
            agent2_id = collision['a2']
            agent1_mdd = mdds[agent1_id]
            agent2_mdd = mdds[agent2_id]
            time_step = collision['time_step']
            if(len(agent1_mdd.levels) - 1 < time_step or len(agent2_mdd.levels) - 1 < time_step):
                continue
            if(doesContainCardinalConflict(agent1_mdd, agent2_mdd)):
                return collision
        #otherwise return random collision
        return collisions[0]