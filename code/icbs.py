import time as timer
import heapq
from common_for_search import compute_heuristics,get_sum_of_cost
from single_agent_planner import a_star
from cbs_utilities import detect_collisions, disjoint_splitting, standard_splitting, paths_violate_constraint, doesContainCardinalConflict, areAgentsDependent, getBetterCollision
from minimumVertexCover import minimumVertexCover, minimumVertexCoverHelper

# class ICBSSolver(object):
#     """The high-level search of CBS."""

#     def __init__(self, my_map, starts, goals):
#         """my_map   - list of lists specifying obstacle positions
#         starts      - [(x1, y1), (x2, y2), ...] list of start locations
#         goals       - [(x1, y1), (x2, y2), ...] list of goal locations
#         """

#         self.my_map = my_map
#         self.starts = starts
#         self.goals = goals
#         self.num_of_agents = len(goals)

#         self.num_of_generated = 0
#         self.num_of_expanded = 0
#         self.CPU_time = 0

#         self.open_list = []

#         # compute heuristics for the low-level search
#         self.heuristics = []
#         for goal in self.goals:
#             self.heuristics.append(compute_heuristics(my_map, goal))

#     def push_node(self, node):
#         heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
#         print("Generate node {}".format(self.num_of_generated))
#         self.num_of_generated += 1

#     def pop_node(self):
#         _, _, id, node = heapq.heappop(self.open_list)
#         print("Expand node {}".format(id))
#         self.num_of_expanded += 1
#         return node

#     def find_solution(self, disjoint=False, h=0):
#         """ Finds paths for all agents from their start locations to their goal locations

#         disjoint    - use disjoint splitting or not
#         """

#         self.start_time = timer.time()
#         root = {'cost': 0,
#                 'constraints': [],
#                 'paths': [],
#                 'collisions': [], 
#                 'mdds': [],
#                 'h_value': 0}
#         for i in range(self.num_of_agents):  # Find initial path for each agent
#             path, mdd = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
#                           i, root['constraints'], isMDD=True)
         
#             if path is None:
#                 raise BaseException('No solutions')
#             root['paths'].append(path)
#             root['mdds'].append(mdd)

#         # print(root['mdds'].levels)

        
#         root['cost'] = get_sum_of_cost(root['paths'])
#         root['collisions'] = detect_collisions(root['paths'])
#         self.push_node(root)
      
#         ##############################
#         # High-Level Search
#         # Repeat the following as long as the open list is not empty:
#         #   1. Get the next node from the open list (you can use self.pop_node()
#         #   2. If this node has no collision, return solution
#         #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
#         #      standard_splitting function). Add a new child node to your open list for each constraint
        
#         while (len(self.open_list) > 0):
         
#             parent_node = self.pop_node()
#             if (len(parent_node['collisions']) == 0): #if no collisions return paths    
#                 self.print_results(parent_node)
#                 return parent_node['paths']
            
            
#             collision = getBetterCollision(parent_node['collisions'], parent_node['mdds'])

#             #splitting the collision 
#             constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

#             for constraint in constraints:
#                 #create an empty node Q
                
#                 Q = {
#                     'cost': 0,
#                     'constraints': [],
#                     'paths': [],
#                     'collisions': [],
#                     'mdds': [],
#                     'h_value': 0
#                     }
              
              
#                 #Copy all constraints from the parent node and add additional constraint     
#                 Q['constraints'] = parent_node['constraints'].copy()           #deep copy 
#                 Q['constraints'].append(constraint)                    
#                 Q['paths'] = parent_node['paths'].copy()
#                 Q['mdds'] = parent_node['mdds'].copy()
               
#                 ai = constraint['agent']
                
#                 #build new path with the new constraint 
#                 path, mdd = a_star(  my_map = self.my_map,
#                                 start_loc = self.starts[ai], 
#                                 goal_loc = self.goals[ai], 
#                                 h_values = self.heuristics[ai],
#                                 agent = ai,
#                                 constraints = Q['constraints'],
#                                 isMDD = True)
#                 is_path_found = True
                
#                 if (path is not None):
#                     if(constraint['positive']):
#                         broken_agents_paths = paths_violate_constraint(parent_node['paths'], constraint)    
#                         for agent in broken_agents_paths:
                         
#                             #generate new path for the broken agent's paths 
#                             updated_path, updated_mdd = a_star   (my_map = self.my_map,
#                                                     start_loc = self.starts[agent], 
#                                                     goal_loc = self.goals[agent], 
#                                                     h_values = self.heuristics[agent],
#                                                     agent = agent,
#                                                     constraints = Q['constraints'], 
#                                                     isMDD= True)
#                             #break if updated path doesn't exist
#                             if (updated_path is None):
#                                 is_path_found = False
#                                 break
#                             #else add the path to high-level node
#                             Q['paths'][agent] = updated_path
#                             Q['mdds'][agent] = updated_mdd
                                        
                            
#                     Q['paths'] [ai] = path
#                     Q['mdds'][ai] = mdd
#                     Q['collisions'] = detect_collisions(Q['paths'])
#                     Q['cost'] = get_sum_of_cost(Q['paths'])
                    
#                     if(is_path_found == True):
#                         self.push_node(Q)
               
#         raise BaseException("No solutions")
       
#     def print_results(self, node):
#         print("\n Found a solution! \n")
#         CPU_time = timer.time() - self.start_time
#         print("CPU time (s):    {:.2f}".format(CPU_time))
#         print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
#         print("Expanded nodes:  {}".format(self.num_of_expanded))
#         print("Generated nodes: {}".format(self.num_of_generated))


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
        heapq.heappush(self.open_list, (node['cost']+node['h_value'], len(node['collisions']), self.num_of_generated, node['h_value'], node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, _, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def computeInformedHeuristics(self, collisions, curr, parent, heuristicType):
      
        num_of_CGedges = None
        HG = [0]*(self.num_of_agents * self.num_of_agents)   # heuristic graph
        h = -1
        if heuristicType == 0:
            h = 0
            return True, h

        elif heuristicType == 1: #CG
            CG, num_of_CGedges = self.buildCardinalConflictGraph(collisions, HG, curr['mdds'])

            if parent is None:  
                h = minimumVertexCoverHelper(CG, self.num_of_agents)
            else:
                h = minimumVertexCover(CG, parent['h_value'], self.num_of_agents, num_of_CGedges)
        elif heuristicType == 2: #DG
            DG, num_of_DGedges = self.buildDependencyGraph(collisions, HG, curr['mdds'])
            
            if parent is None:  
                h = minimumVertexCoverHelper(DG, self.num_of_agents)
            else:
                h = minimumVertexCover(DG, parent['h_value'], self.num_of_agents, num_of_DGedges)
        if h < 0:
            return False, 0
        return True, max(h, curr['h_value'])

    def buildCardinalConflictGraph(self, collisions, CG, mdds):
        num_of_CGedges = 0
        for collision in collisions:
           
            a1 = collision['a1']
            a2 = collision['a2']
            mdd1 = mdds[a1]
            mdd2 = mdds[a2]
            
            if doesContainCardinalConflict(mdd1, mdd2):
                if CG[a1 * self.num_of_agents + a2] == 0:
                    CG[a1 * self.num_of_agents + a2] = 1
                    CG[a2 * self.num_of_agents + a1] = 1
                    num_of_CGedges += 1
        return CG, num_of_CGedges
    
    def buildDependencyGraph(self, collisions, DG, mdds):
        num_of_DGedges = 0

        for collision in collisions:
           
            a1 = collision['a1']
            a2 = collision['a2']
            mdd1 = mdds[a1]
            mdd2 = mdds[a2]
            
            if areAgentsDependent(mdd1, mdd2):
                if DG[a1 * self.num_of_agents + a2] == 0:
                    DG[a1 * self.num_of_agents + a2] = 1
                    DG[a2 * self.num_of_agents + a1] = 1
                    num_of_DGedges += 1
        return DG, num_of_DGedges

    def find_solution(self, disjoint=False, h=0):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'mdds': [],
                'h_value': 0}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path, mdd = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                               i, root['constraints'], isMDD=True)

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            root['mdds'].append(mdd)

        # print(root['mdds'].levels)
        root['collisions'] = detect_collisions(root['paths'])
        root['cost'] = get_sum_of_cost(root['paths'])
        success, root['h_value'] = self.computeInformedHeuristics(root['collisions'], root, None, h)
        print('root h_value: ', root['h_value'])
        
        if not success:
            print('h < 0')

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

            # TODO take the best collison to resolve

            collision = getBetterCollision(parent_node['collisions'], parent_node['mdds'])

            # collision = parent_node['collisions'][0] #take one collision

            # splitting the collision
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                # create an empty node Q

                Q = {
                    'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': [],
                    'mdds': [],
                    'h_value': 0
                }

                # Copy all constraints from the parent node and add additional constraint
                Q['constraints'] = parent_node['constraints'].copy()  # deep copy
                Q['constraints'].append(constraint)
                Q['paths'] = parent_node['paths'].copy()
                Q['mdds'] = parent_node['mdds'].copy()

                ai = constraint['agent']

                # build new path with the new constraint
                path, mdd = a_star(my_map=self.my_map,
                                   start_loc=self.starts[ai],
                                   goal_loc=self.goals[ai],
                                   h_values=self.heuristics[ai],
                                   agent=ai,
                                   constraints=Q['constraints'],
                                   isMDD=True)
                is_path_found = True

                if (path is not None):
                    if (constraint['positive']):
                        broken_agents_paths = paths_violate_constraint(parent_node['paths'], constraint)
                        for agent in broken_agents_paths:

                            # generate new path for the broken agent's paths
                            updated_path, updated_mdd = a_star(my_map=self.my_map,
                                                               start_loc=self.starts[agent],
                                                               goal_loc=self.goals[agent],
                                                               h_values=self.heuristics[agent],
                                                               agent=agent,
                                                               constraints=Q['constraints'],
                                                               isMDD=True)
                            # break if updated path doesn't exist
                            if (updated_path is None):
                                is_path_found = False
                                break
                            # else add the path to high-level node
                            Q['paths'][agent] = updated_path
                            Q['mdds'][agent] = updated_mdd

                    Q['paths'][ai] = path
                    Q['mdds'][ai] = mdd
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    success, Q['h_value'] = self.computeInformedHeuristics(Q['collisions'], Q, parent_node, h)
                    if not success:
                        print('h<0')
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