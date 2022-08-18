
from common_for_search import *
class MDDNode ():
 
    def __init__(self, location, parent_node):
        self.level = None 
        
        self.location = location
        self.children = list()
        self.parents = list()
        self.cost = 0               #minimum cost path traversing this node

        if(parent_node is None):
            self.level = 0
        
        else: 
            self.level = parent_node.level + 1
            self.parents.append(parent_node)


    def areMDDsEqual(self, node):
        return (self.location['loc'] == node['loc'] and self.level == node.level)


class MDD():

    def __init__(self, my_map, start_loc, h_values, num_levels, positive_table, negative_table):
        # print("Building an MDD from scratch")
        self.my_map = my_map
        self.start_loc = start_loc
        # print("start loc", start_loc)
        self.open_list = []
        self.closed_list = []
        self.levels = [[] for _ in range(num_levels)]

        self.h_values = h_values
        self.num_levels = num_levels
        self.positive_table = positive_table
        self.negative_table = negative_table

    def buildMDD(self):
        root = MDDNode(self.start_loc, None)
        root.cost = self.num_levels - 1 
        
        self.open_list.append(root)
        self.closed_list.append(root)
        while (len(self.open_list) > 0):
            curr = self.open_list.pop(0)
        
            if(curr.level == self.num_levels - 1):
             
                self.levels[-1].append(curr)
                assert(len(self.open_list) == 0)
                break

    
            heuristicBound = self.num_levels - curr.level - 2
            
            #expanding current node
            for dir in range(5):
                child_loc = move(curr.location, dir)
                ###################Checking if the child node is valid####################
                
                #Check if the child_loc position is not out of the maps' bounds
                if (child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(self.my_map) or child_loc[1] >= len(self.my_map[0])): 
                    continue
        
                #checks if the child_loc in the map is not blocked
                if self.my_map[child_loc[0]][child_loc[1]]:     
                    continue

                #Check if new node satisfies the passed negative constraints, if doesn't -> prune
                if (is_constrained(curr.location, child_loc, curr.level + 1, self.negative_table)):
                    continue
            
                # if (is_constrained(child_loc, curr.location, curr.level + 1, self.negative_table)):
                #     continue

                #Check if new node satisfies the passed positive constraints, if doesn't -> prune
                p_node = {'loc': curr.location}
                c_node = {'loc': child_loc} 
                if (is_positive_satisfied(parent_node = p_node, child_node = c_node, next_time = curr.level + 1, positive_constraints = self.positive_table) == False):
                    continue
              
                if(self.h_values[child_loc] <= heuristicBound):
                    #TODO add child node here
                    child =  self.closed_list[-1]
                    find = False

                    for child in reversed(self.closed_list):
                        if(child.level == curr.level + 1):
                            
                            if(child.location == child_loc):
                                child.parents.append(curr)
                                find = True
                                break

                    if (find is False):
                        childNode = MDDNode(child_loc, curr)
                        childNode.cost = self.num_levels - 1
                        self.open_list.append(childNode)
                        self.closed_list.append(childNode)
        # print("goal node: ", self.levels[-1][0].location)
        
        
        assert(len(self.levels[-1]) == 1)
        
        goal_node = self.levels[-1][-1]
        delete = None
       
        for parent in goal_node.parents:
            if(parent.location == goal_node.location):
                delete = parent
                continue 
            self.levels[self.num_levels - 2].append(parent)
            parent.children.append(goal_node)

        if(delete != None):
            goal_node.parents.remove(delete)

        for t in range (self.num_levels - 2, 0, -1):
            # print("t = ", t)
            for node in self.levels[t]:
                for parent in node.parents: 
                    if (len(parent.children) == 0):
                        self.levels[t - 1].append(parent)
                    parent.children.append(node)
        level_num = 0
        
        for it in self.closed_list:
            if(len(it.children) == 0):
                if(it.level < self.num_levels - 1):
                    del it 
        self.closed_list.clear()
  

              

    def deleteNode():
        return True    
    def goalIsAt():
        return True 
    
        