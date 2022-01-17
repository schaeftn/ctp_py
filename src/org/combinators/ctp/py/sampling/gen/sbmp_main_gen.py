import sys
from functools import partial

from org.combinators.ctp.py.sampling.WAFRRobotStateValidator import WafrRobotStateValidator

sys.path.append("/home/tristan/projects/ctp_py/src")

from org.combinators.ctp.py.sampling.FclMotionValidator import *
from org.combinators.ctp.py.sampling.FclStateValidator import *
from org.combinators.ctp.py.sampling.PathRefinement import *

from enum import Enum
import asyncio

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



class MyMotionValidator(ob.MotionValidator):
    def __init__(self, si, scene_):
        super(MyMotionValidator, self).__init__(si)
        self.scene = scene_
        self.state = ob.State(ob.SE3StateSpace())

    def checkMotion(self, *args, **kwargs):
        s1 = args[0]
        s2 = args[1]
        valid_motion = False

        if args.__len__() == 2:
            valid_motion, min_val = self.scene.check_free_line_3d(
                [s1.getX(), s1.getY(), s1.getZ()],
                [s2.getX(), s2.getY(), s2.getZ()])

        if args.__len__() == 3:
            valid_motion, col_param = self.scene.check_free_line_3d(
                [s1.getX(), s1.getY(), s1.getZ()],
                [s2.getX(), s2.getY(), s2.getZ()])
            min_val = col_param - 0.01 if col_param - 0.01 >= 0.0 else 0.0
            x = s1.getX() + min_val * (s2.getX() - s1.getX())
            y = s1.getY() + min_val * (s2.getY() - s1.getY())
            z = s1.getZ() + min_val * (s2.getZ() - s1.getZ())
            args[2].first.setX(x)
            args[2].first.setY(y)
            args[2].second = min_val

        return valid_motion


# check https://github.com/kucars/laser_collision_detection/blob/1b78fe5a95584d135809b1448d33675bb8fee250/src/laser_obstacle_detect.cpp#L252
# data format, resolution, ansonsten check api fcl...
class plan:
    def __init__(self, planning_time=10.0, simplification_time=2.0):
        self.planning_time = planning_time
        self.simplification_time = simplification_time
        env = "/home/tristan/projects/ctp_py/resources/Robots/RobotEnvironments/env1.stl"
        package = "/home/tristan/projects/ctp_py/resources/Robots/RobotDescriptions/clsrobo_44550_description"
        wafr_validator = WafrRobotStateValidator(package, env)
        space = wafr_validator.getSamplingSpace()

        start_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        start = ob.State(space)
        for index, value in enumerate(start_list):
            start[index] = value

        space.enforceBounds(start())

        goal_list = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        goal = ob.State(space)
        for index, value in enumerate(goal_list):
            goal[index] = value

        space.enforceBounds(goal())

        # space.setStateSamplerAllocator(ob.StateSamplerAllocator(ob.RealVectorStateSampler))

        self.si = ob.SpaceInformation(space)

        self.si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(ob.UniformValidStateSampler))

        self.si.setMotionValidator(ob.DiscreteMotionValidator(self.si))
        #self.si.setMotionValidator(FclMotionValidator(self.si))

        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(partial(wafr_validator.isValid, self.si)))
        #self.si.setStateValidityCheckingResolution(0.01)

        self.ss = og.SimpleSetup(self.si)
        p = og.PRMstar(self.si)
        self.ss.setPlanner(p)
        self.ss.setStartAndGoalStates(start, goal)
        self.ss.setup()
        self.ss.getPlanner().checkValidity()

        self.solve()
        np.set_printoptions(formatter={'float': '{:0.5f}'.format})

    def solve(self):
        solved = self.ss.solve(self.planning_time)
        if solved and self.ss.haveExactSolutionPath():
            print("Found exact solution:")
            self.ss.simplifySolution(self.simplification_time)
        else:
            print("No exact solution found")

        data = ob.PlannerData(self.si)
        self.ss.getPlannerData(data)

        explored_states = []
        print(f"solution path:")

        print(f"""{{"exploredStates" : {explored_states}}}""")

        if self.ss.haveSolutionPath():
            print(f"solution path: {self.ss.getSolutionPath()}")
        else:
            print(f"WARN: no solution path")

    def isStateValid(self, state):
        return self.scene.is_valid([state.getX(), state.getY(), state.getZ()])

    @staticmethod
    def getStateVector(s):
        return [s.getX(), s.getY(), s.getZ()]


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ompl planning')
    parser.add_argument('--computationtime',  default=10.0,help='maximal time allowed for computation', type=float)
    parser.add_argument('--simplificationtime',  default=2.0,help='maximal time allowed for simplification', type=float)

    args = parser.parse_args()
    p = plan(args.computationtime, args.simplificationtime)