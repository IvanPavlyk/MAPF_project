from dis import dis
import time as timer
import heapq
import random
from queue import Queue
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, pop_node, push_node

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

    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
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
        # High-Level Search
        # Repeat the following as long as the open list is not empty:
        #   1. Get the next node from the open list (you can use self.pop_node()
        #   2. If this node has no collision, return solution
        #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #      standard_splitting function). Add a new child node to your open list for each constraint

        while (len(self.open_list) > 0):

            parent_node = self.pop_node()
            if (len(parent_node['collisions']) == 0):  # if no collisions return paths
                self.print_results(parent_node)
                return parent_node['paths']

            collision = parent_node['collisions'][0]  # take one collision
            constraints = constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                # create an empty node Q

                Q = {
                    'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []
                }

                # Copy all constraints from the parent node and add additional constraint
                Q['constraints'] = parent_node['constraints'].copy()  # deep copy
                Q['constraints'].append(constraint)
                Q['paths'] = parent_node['paths'].copy()

                ai = constraint['agent']

                # build new path with the new constraint
                path = a_star(my_map=self.my_map,
                              start_loc=self.starts[ai],
                              goal_loc=self.goals[ai],
                              h_values=self.heuristics[ai],
                              agent=ai,
                              constraints=Q['constraints'])
                is_path_found = True

                if (path is not None):
                    if (constraint['positive']):
                        broken_agents_paths = paths_violate_constraint(parent_node['paths'], constraint)
                        for agent in broken_agents_paths:

                            # generate new path for the broken agent's paths
                            updated_path = a_star(my_map=self.my_map,
                                                  start_loc=self.starts[agent],
                                                  goal_loc=self.goals[agent],
                                                  h_values=self.heuristics[agent],
                                                  agent=agent,
                                                  constraints=Q['constraints'])
                            # break if updated path doesn't exist
                            if (updated_path is None):
                                is_path_found = False
                                break
                            # else add the path to high-level node
                            Q['paths'][agent] = updated_path

                    Q['paths'][ai] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    if (is_path_found == True):
                        self.push_node(Q)

        raise BaseException("No solutions")

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))


class CBSSolverIvan(object):
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
        heapq.heappush(self.open_list, (node['cost'] + node['h_value'], len(node['collisions']), self.num_of_generated, node['h_value'], node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def computeInformedHeuristics(self, collisions, curr, parent, heuristicType):
        # create conflict graph
        num_of_CGedges = None
        HG = [0]*(self.num_of_agents * self.num_of_agents)   # heuristic graph
        h = -1
        if heuristicType == 0:
            h = 0
        elif heuristicType == 1: #CG
            CG, num_of_CGedges = self.buildCardinalConflictGraph(collisions, HG, curr['mdds'])
            # Minimum Vertex Cover
            if parent is None:  # when we are allowed
                # to replan for multiple agents, the incremental method is not correct any longer.
                h = self.minimumVertexCoverHelper(HG)
            else:
                #assert len(curr['paths']) == 1
                h = self.minimumVertexCover(HG, parent['h_value'], self.num_of_agents, num_of_CGedges)
        #elif heuristicType == 2: #DG
        #    if not buildDependenceGraph(curr, HG, num_of_CGedges):
        #        return False
        #    # Minimum Vertex Cover
        #    if curr.parent is None:  # when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
        #        h = minimumVertexCover(HG)
        #    else:
        #        h = minimumVertexCover(HG, curr.parent.h_val, num_of_agents, num_of_CGedges)
        #elif heuristicType == 3: #WDG
        #    if not buildWeightedDependencyGraph(curr, HG):
        #        return False
        #    h = minimumWeightedVertexCover(HG)
        if h < 0:
            return False, 0
        return True, max(h, curr['h_value'])

    def buildCardinalConflictGraph(self, collisions, CG, mdds):
        num_of_CGedges = 0
        for collision in collisions:
            print("a2 ", collision['a2'])
            print("mdds ", mdds)
            print("mdds length", len(mdds))
            if self.is_cardinal_conflict(collision, mdds) == 'cardinal':
                a1 = collision['a1']
                a2 = collision['a2']
                if CG[a1 * self.num_of_agents + a2] == 0:
                    CG[a1 * self.num_of_agents + a2] = 1
                    CG[a2 * self.num_of_agents + a1] = 1
                    num_of_CGedges += 1
        return CG, num_of_CGedges

    def minimumVertexCoverHelper(self, CG):
        rst = 0
        done = [False] * self.num_of_agents
        i = 0
        while i < self.num_of_agents:
            if done[i]:
                continue
            indices = []
            Q = Queue()
            Q.put(i)
            done[i] = True
            while not Q.empty():
                j = Q.get()
                #Q.pop()
                indices.append(j)
                k = 0
                while k < self.num_of_agents:
                    if CG[j * self.num_of_agents + k] > 0:
                        if not done[k]:
                            Q.put(k)
                            done[k] = True
                    elif CG[k * self.num_of_agents + j] > 0:
                        if not done[k]:
                            Q.put(k)
                            done[k] = True
                    k += 1
            if len(indices) == 1:  # one node -> no edges -> mvc = 0
                continue
            elif len(indices) == 2:  # two nodes -> only one edge -> mvc = 1
                rst += 1  # add edge weight
                continue

            subgraph = [0] * (len(indices) * len(indices))
            num_edges = 0
            j = 0
            while j < len(indices):
                k = j + 1
                while k < len(indices):
                    subgraph[j * len(indices) + k] = CG[indices[j] * self.num_of_agents + indices[k]]
                    subgraph[k * len(indices) + j] = CG[indices[k] * self.num_of_agents + indices[j]]
                    if subgraph[j * len(indices) + k] > 0:
                        num_edges += 1
                    k += 1
                j += 1
            if len(indices) > 8:
                rst += self.greedyMatching(subgraph, len(indices))
            else:
                i = 1
                while i < len(indices):
                    if self.KVertexCover(subgraph, len(indices), num_edges, i, len(indices)):
                        rst += i
                        break
                    i += 1
            i += 1
        return rst

    def minimumVertexCover(self, CG, old_mvc, cols, num_of_CGedges):
        assert old_mvc >= 0
        rst = 0
        if num_of_CGedges < 2:
            return num_of_CGedges
        # Compute #CG nodes that have edges
        num_of_CGnodes = 0
        for i in range(0, cols):
            for j in range(0, cols):
                if CG[i * cols + j] > 0:
                    num_of_CGnodes += 1
                    break
        if num_of_CGnodes > 8:
            return self.minimumVertexCover(CG)
        else:
            if self.KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols):
                rst = old_mvc - 1
            elif self.KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols):
                rst = old_mvc
            else:
                rst = old_mvc + 1
        return rst

    def greedyMatching(self, CG, cols):
        rst = 0
        used = [False] * cols
        while True:
            maxWeight = 0
            ep1 = None
            ep2 = None
            for i in range(0, cols):
                if used[i]:
                    continue
                for j in range(i + 1, cols):
                    if used[j]:
                        continue
                    elif maxWeight < CG[i * cols + j]:
                        maxWeight = CG[i * cols + j]
                        ep1 = i
                        ep2 = j
            if maxWeight == 0:
                return rst
            rst += maxWeight
            used[ep1] = True
            used[ep2] = True

    def KVertexCover(self, CG, num_of_CGnodes, num_of_CGedges, k, cols):
        if num_of_CGedges == 0:
            return True
        elif num_of_CGedges > k * num_of_CGnodes - k:
            return False
        node = [0 for _ in range(2)]
        flag = True
        i = 0
        while i < cols - 1 and flag:  # to find an edge
            j = i + 1
            while j < cols and flag:
                if CG[i * cols + j] > 0:
                    node[0] = i
                    node[1] = j
                    flag = False
                j += 1
            i += 1
        for i in range(0, 2):
            CG_copy = CG.copy()
            #CG_copy.assign(CG.cbegin(), CG.cend())
            num_of_CGedges_copy = num_of_CGedges
            for j in range(0, cols):
                if CG_copy[node[i] * cols + j] > 0:
                    CG_copy[node[i] * cols + j] = 0
                    CG_copy[j * cols + node[i]] = 0
                    num_of_CGedges_copy -= 1
            if self.KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols):
                return True
        return False

    def is_cardinal_conflict(self, collision, mdds):
        a1 = collision['a1']
        a2 = collision['a2']
        t = collision['time_step']
        loc = collision['loc']
        mdd1 = mdds[a1]
        mdd2 = mdds[a2]

        # vertex conflict
        # The collision is derived from path, and path is derived from mdd. Hence the conflict location is guaranteed in both mdd1 and mdd2 at timestep t.
        # We only need to check if there is only one node in mdd1 and mdd2 at timestep t
        if len(loc) == 1:
            if len(self.get_mdd_nodes(mdd1, t)) == len(self.get_mdd_nodes(mdd2, t)) == 1:
                return True
        # edge conflict
        if len(loc) == 2:  # timestep t > 0 is guaranteed
            if len(self.get_mdd_nodes(mdd1, t - 1)) == len(self.get_mdd_nodes(mdd1, t)) == len(self.get_mdd_nodes(mdd2, t - 1)) == len(
                    self.get_mdd_nodes(mdd2, t)) == 1:
                return True
        return False

    def get_mdd_nodes(self, mdd, time):
        if time < 0:
            return mdd[0]
        elif time < len(mdd):
            return mdd[time]
        else:
            return mdd[-1]

    def build_mdd(self, paths):
        path_len = len(paths[0])
        i= 0
        for path in paths:
            print("path ",i, " ", path)
            i += 1
        print("path_len ", path_len)
        print("paths length", len(paths))
        mdd = []

        #???????????path_len???????????instead of path_len+1
        for ts in range(path_len):
            locs = [p[ts] for p in paths]
            locs_set = set(locs)
            matchings = {
                l: [i for i, x in enumerate(locs) if x == l] for l in locs_set
            }
            mdd_ts = [{
                'loc': l,
                'child': list(set([paths[i][ts + 1] for i in matchings[l]])) if ts != path_len - 1 else [None]
            } for l in locs_set]

            mdd.append(mdd_ts)

        return mdd

    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'h_value': 0,
                'mdds': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path, mddi = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            root['mdds'].append(mddi)
         
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        success, root['h_value'] = self.computeInformedHeuristics(root['collisions'], root, root, h)
        if not success:
            print("h < 0")
        self.push_node(root)

        ##############################
        # High-Level Search
        # Repeat the following as long as the open list is not empty:
        #   1. Get the next node from the open list (you can use self.pop_node()
        #   2. If this node has no collision, return solution
        #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #      standard_splitting function). Add a new child node to your open list for each constraint
        
        while (len(self.open_list) > 0):   
            node = self.pop_node()
            if (len(node['collisions']) == 0): #if no collisions return paths    
                #print("PAths", node['paths'])
                #print("Sum of costs", node['cost'])
                self.print_results(node)
                return node['paths']
         
            parent_node = self.pop_node()
            if (len(parent_node['collisions']) == 0): #if no collisions return paths    
                self.print_results(parent_node)
                return parent_node['paths']       
            collision = parent_node['collisions'][0] #take one collision
            constraints = constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                #create an empty node Q
                
                Q = {
                    'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': [],
                    'h_value': 0,
                    'mdds': node['mdds'].copy()
                    }
                #Copy all constraints from the parent node and add additional constraint 
                Q['constraints'] = node['constraints'].copy()           #deep copy
                Q['constraints'].append(constraint)
                Q['paths'] = node['paths'].copy()  
                ai = constraint['agent']
                
                #build new path with the new constraint 
                path, mddi = a_star(  my_map = self.my_map,
                                start_loc = self.starts[ai], 
                                goal_loc = self.goals[ai], 
                                h_values = self.heuristics[ai],
                                agent = ai,
                                constraints = Q['constraints'])
                is_path_found = True
                if (path is not None):
                    if(constraint['positive']):
                        broken_agents_paths = paths_violate_constraint(parent_node['paths'], constraint)    
                        for agent in broken_agents_paths:
                         
                            #generate new path for the broken agent's paths 
                            updated_path = a_star   (my_map = self.my_map,
                                                    start_loc = self.starts[agent], 
                                                    goal_loc = self.goals[agent], 
                                                    h_values = self.heuristics[agent],
                                                    agent = agent,
                                                    constraints = Q['constraints'])
                            #break if updated path doesn't exist
                            if (updated_path is None):
                                is_path_found = False
                                break
                            #else add the path to high-level node
                            Q['paths'][agent] = updated_path
                            Q['mdds'][agent] = mddi
                    
                    if(is_path_found == False):
                        continue        

                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    success, Q['h_value'] = self.computeInformedHeuristics(Q['collisions'], Q, node, h)
                    if not success:
                        print("h < 0")
                    self.push_node(Q)
                else:
                    print("Constraint", constraint)
                    print("Q constraints", Q['constraints'])
                    print("Start",  self.starts[ai])
                    print("Goal ", self.goals[ai])
                    print("path is non")                                                          
                    Q['paths'] [ai] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    if(is_path_found == True):
                        self.push_node(Q)
               
        raise BaseException("No solutions")
     
    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

       
class ICBSSolver(object):
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

    def doesContainCardinalConflict(self, mdd1, mdd2):
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

    def getBetterCollision(self, collisions, mdds): 
        for collision in collisions:
            agent1_id = collision['a1']
            agent2_id = collision['a2']
            agent1_mdd = mdds[agent1_id]
            agent2_mdd = mdds[agent2_id]
            if(self.doesContainCardinalConflict(agent1_mdd, agent2_mdd)):
                return collision
        #otherwise return the first collision
        return collisions[0]



    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [], 
                'mdds': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path, mdd = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'], isMDD=True)
         
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            root['mdds'].append(mdd)

        # print(root['mdds'].levels)

        
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
      
        ##############################
        # High-Level Search
        # Repeat the following as long as the open list is not empty:
        #   1. Get the next node from the open list (you can use self.pop_node()
        #   2. If this node has no collision, return solution
        #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #      standard_splitting function). Add a new child node to your open list for each constraint
        
        while (len(self.open_list) > 0):
         
            parent_node = self.pop_node()
            if (len(parent_node['collisions']) == 0): #if no collisions return paths    
                self.print_results(parent_node)
                return parent_node['paths']
            
            #TODO take the best collison to resolve 
            
            collision = self.getBetterCollision(parent_node['collisions'], parent_node['mdds'])
            
            # collision = parent_node['collisions'][0] #take one collision

            #splitting the collision 
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                #create an empty node Q
                
                Q = {
                    'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': [],
                    'mdds': []
                    }
              
              
                #Copy all constraints from the parent node and add additional constraint     
                Q['constraints'] = parent_node['constraints'].copy()           #deep copy 
                Q['constraints'].append(constraint)                    
                Q['paths'] = parent_node['paths'].copy()
                Q['mdds'] = parent_node['mdds'].copy()
               
                ai = constraint['agent']
                
                #build new path with the new constraint 
                path, mdd = a_star(  my_map = self.my_map,
                                start_loc = self.starts[ai], 
                                goal_loc = self.goals[ai], 
                                h_values = self.heuristics[ai],
                                agent = ai,
                                constraints = Q['constraints'],
                                isMDD = True)
                is_path_found = True
                
                if (path is not None):
                    if(constraint['positive']):
                        broken_agents_paths = paths_violate_constraint(parent_node['paths'], constraint)    
                        for agent in broken_agents_paths:
                         
                            #generate new path for the broken agent's paths 
                            updated_path, updated_mdd = a_star   (my_map = self.my_map,
                                                    start_loc = self.starts[agent], 
                                                    goal_loc = self.goals[agent], 
                                                    h_values = self.heuristics[agent],
                                                    agent = agent,
                                                    constraints = Q['constraints'], 
                                                    isMDD= True)
                            #break if updated path doesn't exist
                            if (updated_path is None):
                                is_path_found = False
                                break
                            #else add the path to high-level node
                            Q['paths'][agent] = updated_path
                            Q['mdds'][agent] = updated_mdd
                                        
                            
                    Q['paths'] [ai] = path
                    Q['mdds'][ai] = mdd
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    
                    if(is_path_found == True):
                        self.push_node(Q)
               
        raise BaseException("No solutions")
       

           
           
            
    
    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
