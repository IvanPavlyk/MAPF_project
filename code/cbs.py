
import time as timer
import heapq

from queue import Queue
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from cbs_utilities import detect_collisions, disjoint_splitting, standard_splitting, paths_violate_constraint


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
                CPU_time = timer.time() - self.start_time
                return CPU_time, self.num_of_expanded, self.num_of_generated, parent_node['paths']

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