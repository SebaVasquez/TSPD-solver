
from gurobipy import Model as GRBModel

class Model:
    def __init__(self, instance):
        self.instance = instance
        self.model = GRBModel()
        self.variables = dict()
        self.constr = dict()
    
    def _reset(self):
        self.model = GRBModel()