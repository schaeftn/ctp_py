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
        print("PR INIT")

    def sample(self, *args, **kwargs):
        r = self.rng_.uniformReal(0, 1)
        upper_index = random.choice(range(len(self.path)-1))+1
        lower_index = upper_index - 1
        (p1, p2) = (self.path[lower_index], self.path[upper_index])

        point = r * p1 + r * (p2 - p1)
        sample_point = [val + self.rng_.gaussian(0, distance(p1, p2) / 2) for val in point]

        print(f"pDistance: {distance(p1, p2)}")
        print(f"Distance: {distance(point, sample_point)}")

        ##self.rng_.quaternion(args[0].rotation())

        args[0].setX(sample_point[0])
        args[0].setY(sample_point[1])
        args[0].setZ(sample_point[2])
        args[0].rotation().z = 0.0

        return self.si_.isValid(*args, **kwargs)

    def sampleNear(self, *args, **kwargs):
        print(f"sampleNear called")
        return False

    def sampleUniformNear(self, *args, **kwargs):
        print(f"uniNear")
        return self.sample(*args,  **kwargs)

    def rng_quaternion(self):
        x0 = self.rng_.uniform01()
        r1 = math.sqrt(1.0 - x0)
        r2 = math.sqrt(x0)
        t1 = 2.0 * math.pi * self.rng_.uniform01()
        t2 = 2.0 * math.pi * self.rng_.uniform01()
        c1 = math.cos(t1)
        s1 = math.sin(t1)
        c2 = math.cos(t2)
        s2 = math.sin(t2)
        return [s1 * r1, c1 * r1, s2 * r2, c2 * r2]

class PathRefinementSpSampler(ob.CompoundStateSampler):
    def __init__(self, *args, **kwargs):
        print("PR init start")
        sp = args[0]
        super(PathRefinementSpSampler, self).__init__(sp)
        self.si_ = sp
        self.name_ = "Path refinement sampler"
        self.rng_ = ou.RNG()
        self.path = path_data.path_list
        print("PR init stop")

    def sample(self, *args, **kwargs):
        print("PR sample")
        r = self.rng_.uniformReal(0, 1)
        upper_index = random.choice(range(len(self.path)-1))+1
        lower_index = upper_index - 1
        (p1, p2) = (self.path[lower_index], self.path[upper_index])

        point = r * p1 + r * (p2 - p1)
        sample_point = [val + self.rng_.gaussian(0, distance(p1, p2) / 2) for val in point]

        print(f"pDistance: {distance(p1, p2)}")
        print(f"Distance: {distance(point, sample_point)}")

        ##self.rng_.quaternion(args[0].rotation())

        args[0].setX(sample_point[0])
        args[0].setY(sample_point[1])
        args[0].setZ(sample_point[2])
        quats = self.rng_quaternion()
        args[0].rotation().x = quats[0]
        args[0].rotation().y = quats[1]
        args[0].rotation().z = quats[2]
        args[0].rotation().w = quats[3]
        print(f"{args[0]}")
        print(f"{args[0].rotation()}")

    def sampleNear(self, *args, **kwargs):
        print(f"sampleNear called")
        return False

    def sampleUniformNear(self, *args, **kwargs):
        print(f"uniNearSpSampl")
        return self.sample(*args,  **kwargs)

    def rng_quaternion(self):
        x0 = self.rng_.uniform01()
        r1 = math.sqrt(1.0 - x0)
        r2 = math.sqrt(x0)
        t1 = 2.0 * math.pi * self.rng_.uniform01()
        t2 = 2.0 * math.pi * self.rng_.uniform01()
        c1 = math.cos(t1)
        s1 = math.sin(t1)
        c2 = math.cos(t2)
        s2 = math.sin(t2)
        return [s1 * r1, c1 * r1, s2 * r2, c2 * r2]

class PathRefinementSampler2(ob.CompoundStateSampler):
    def __init__(self, si):
        super(PathRefinementSampler2, self).__init__(si)
        self.si_ = si
        self.name_ = "Path refinement sampler"
        self.rng_ = ou.RNG()
        self.path = path_data.path_list
        self.addSampler(PathRefinementSampler(si), 1.0)
        self.addSampler(ob.UniformStateSampler, 1.0)
        print("PR INIT")

    def sample(self, *args, **kwargs):
        r = self.rng_.uniformReal(0, 1)
        upper_index = random.choice(range(len(self.path)-1))+1
        lower_index = upper_index - 1
        (p1, p2) = (self.path[lower_index], self.path[upper_index])

        point = r * p1 + r * (p2 - p1)
        sample_point = [val + self.rng_.gaussian(0, distance(p1, p2) / 2) for val in point]

        print(f"Distance: {distance(point, sample_point)}")

        args[0].setX(sample_point[0])
        args[0].setY(sample_point[1])
        args[0].setZ(sample_point[2])

        return self.si_.isValid(*args, **kwargs)

    def sampleNear(self, *args, **kwargs):
        print(f"sampleNear called")
        return False

    def sampleUniformNear(self, *args, **kwargs):
        print(f"uniNear")
        return self.sample(*args,  **kwargs)


def distance(p1, p2):
    return math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(np.array(p1), np.array(p2))]))
