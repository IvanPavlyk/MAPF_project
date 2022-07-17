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
        constraints = []
        #Task 1.2, add a vertex constraint that prohibits agent 0 from being at its goal cell (1,5) at time step 4
        # constraints.append({'agent':0, \
        #                     'loc': [(1,5)], \
        #                     'time_step': 4})
        #Task 1.3, add an edge costraint that prohibits agent 1 from moving from (1,2) to (1,3) from timestep 0 to 1
        # constraints.append({'agent': 1, \
        #                     'loc': [(1,2),(1,3)], \
        #                     'time_step': 1})
        #Task 1.4, add constraint that prohibits agent 0 from being at its goal cell (1,5) at timestep 10
        # constraints.append({'agent': 0, \
        #                     'loc': [(1,5)], \
        #                     'time_step': 10})
        #Task 1.5 hand design constraints that allows to find collision free paths with minimal sum of path length
        # constraints = [{'agent': 1, \
        #                 'loc': [(1,4)], \
        #                 'time_step': 2},\
        
        #                 {'agent': 1, \
        #                 'loc': [(1,2)], \
        #                 'time_step': 2}]
        
        #Task 2.3 calculate the number navigiatable cells in the map
        max_map_cells = 0
        for i in range (len(self.my_map)):
            for j in range (len(self.my_map[0])):
                if (self.my_map[i][j] != True):
                    max_map_cells += 1
        ############################################################
        
        
        time_total_prev_agent = 0
        max_time_steps = max_map_cells
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            max_time_steps = max_map_cells
            
            max_time_steps += time_total_prev_agent
            if(len(path) >= max_time_steps):
                raise BaseException('No solutions')
            
            
           
            # print("Max time steps", max_time_steps)
           
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            #The solution path of the current agent would the set of constraints for next agents' plans
            
           
            #Task 2, creating constraints 
            for agent_num in range (i + 1, self.num_of_agents):
                path_len = len(path)
                

                for time_step in range (path_len):
                    #Task 2.1 Adding Vertex Constraints 
                    constraints.append({'agent': agent_num, 
                                        'loc': [path[time_step]],
                                        'time_step': time_step
                                        })
                    #Task 2.2 Adding Edge Constraints 
                    constraints.append({'agent': agent_num, 
                                        'loc': [path[time_step], path[time_step-1]],
                                        'time_step': time_step
                                        })

                #Task 2.3 add agent's goal position as a constraint to all future agents
                for time_step_future in range (path_len, max_time_steps ):
                    constraints.append({'agent': agent_num, 
                                        'loc': [path[path_len-1]],
                                        'time_step': time_step_future
                                        })
            time_total_prev_agent = len(path)

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
