
class Solution:
    def __init__(self, instance):
        self.instance = instance
        self.truck_arcs = list()
        self.operations = list()
        self.start_time = 0
        self.integer_solutions = 0
        self.incumbent_time = None
        self.Q = dict()
        self.shortcuts = dict()
        self.disconnected_components = 0
        self.disconnected_components_time_list = list()
        self.disconnected_components_mincut = 0
        self.disconnected_components_mincut_time_list = list()
        self.IOC = 0
        self.IOC_time_list = list()
        self.feasibility_cuts = 0

    @property
    def drone_arcs(self):
        arcs = list()
        for (i, k, j) in self.operations:
            arcs += [(i, k), (k, j)]

        return arcs
        
    @property
    def disconnected_components_time(self):
        return sum(self.disconnected_components_time) // len(self.disconnected_components_time)

    @property
    def disconnected_components_mincut_time(self):
        return sum(self.disconnected_components_mincut_time) // len(self.disconnected_components_mincut_time)

    @property
    def IOC_time(self):
        return sum(self.IOC_time) // len(self.IOC_time)