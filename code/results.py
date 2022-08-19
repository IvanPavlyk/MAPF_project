class Results(object):
    def __init__(self):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.cpu_time = 0
        self.expanded = 0
        self.generated = 0

    def addValues(self, cpu_time, expanded, generated):
        self.cpu_time += cpu_time
        self.expanded += expanded
        self.generated += generated

    def printResults(self, num_of_experiments):
        print("Time: ", self.cpu_time)
        print("Expanded nodes: ", self.expanded)
        print("Generated nodes: ", self.generated)
        print("------------------")
        print("Average time: ", self.cpu_time/num_of_experiments)
        print("Average expanded nodes: ", self.expanded/num_of_experiments)
        print("Average generated nodes: ", self.generated/num_of_experiments)
