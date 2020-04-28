import random
import math
import numpy as np

from org.combinators.ctp.py.sampling.gen import path_data

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys


class PathRefinementSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super(PathRefinementSampler, self).__init__(si)
        self.si_ = si
        self.name_ = "Path refinement sampler"
        self.rng_ = ou.RNG()
        self.path = path_data.path_list

    def sample(self, *args, **kwargs):
        r = self.rng_.uniformReal(0, 1)
        upper_index = random.choice(range(len(self.path)-1))+1
        lower_index = upper_index - 1
        (p1, p2) = (self.path[lower_index], self.path[upper_index])

        point = r * p1 + r * (p2 - p1)
        sample_point = [val + self.rng_.gaussian(0, distance(p1, p2) / 2) for val in point]

        args[0].setX(sample_point[0])
        args[0].setY(sample_point[1])
        args[0].setZ(sample_point[2])

        return self.si_.isValid(*args, **kwargs)

    def sampleNear(self, *args, **kwargs):
        print(f"sampleNear called")
        return False


def distance(p1, p2):
    return math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(np.array(p1), np.array(p2))]))
