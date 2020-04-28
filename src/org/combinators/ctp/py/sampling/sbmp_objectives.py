import sys

sys.path.append("/home/tristan/projects/ctp_sampling_py/src")

import numpy as np

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

import math


class WeightedEuclideanCostObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, start_3d, end_3d,weights):
        super(WeightedEuclideanCostObjective, self).__init__(si)
        self.start_ = np.array(start_3d)
        self.end_ = np.array(end_3d)
        self.weights_ = np.array(weights)

    def stateCost(self, s):
        return self.getSpaceInformation().getStateValidityChecker().clearance(s)
        # return ob.Cost(clearance_s(x, y, z, self.start_, self.end_, self.alpha_, self.beta_))

    def motionCost(self, *args, **kwargs):
        s1_values = np.array([args[0][0], args[0][1], args[0][2], args[0][3], args[0][4]])
        s2_values = np.array([args[1][0], args[1][1], args[1][2], args[1][3], args[1][4]])
        s1 = np.multiply(s1_values, self.weights_)
        s2 = np.multiply(s2_values, self.weights_)

        distance = math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(s1, s2)]))

        return distance