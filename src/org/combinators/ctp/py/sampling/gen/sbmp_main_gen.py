import sys

sys.path.append("/home/tristan/projects/ctp_py/src")

from org.combinators.ctp.py.sampling.WAFRRobotStateValidator import *
from org.combinators.ctp.py.sampling.FclMotionValidator import *
from org.combinators.ctp.py.sampling.FclStateValidator import *
from org.combinators.ctp.py.sampling.PathRefinement import *
from functools import partial

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


class plan:
    def __init__(self, planning_time=10.0, simplification_time=2.0):
        self.planning_time = planning_time
        self.simplification_time = simplification_time
                # State Validator Arguments
        env = "/home/tristan/projects/ctp_py/resources/Robots/RobotEnvironments/env4.stl"
        package = "/home/tristan/projects/ctp_py/resources/Robots/RobotDescriptions/clsrobo_90441_description"
        
        wafr_validator = WafrRobotStateValidator(package, env)
        space = wafr_validator.getSamplingSpace()

        # set the bounds
        

        #start state
        start_list = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start = ob.State(space)
        for index, value in enumerate(start_list):
            start[index] = value

        space.enforceBounds(start())

        goal_list = [0.680678,-0.855211,-0.15708,0.0349066,-0.698132,1.3439,-1.5708,-0.523599,0.715585,0.0]
        goal = ob.State(space)
        for index, value in enumerate(goal_list):
            goal[index] = value

        space.enforceBounds(goal())
        

        
        self.si = ob.SpaceInformation(space)
        self.si.setMotionValidator(ob.DiscreteMotionValidator(self.si))
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(partial(wafr_validator.isValid, self.si)))
        
        self.si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(ob.GaussianValidStateSampler))

        asd = ob.StateValidityCheckerFn(partial(wafr_validator.isValid, self.si))

        self.ss = og.SimpleSetup(self.si)
        p = og.PRM(self.si)
        self.ss.setPlanner(p)
        self.ss.setStartAndGoalStates(start, goal)
        self.ss.setup()
        self.ss.getPlanner().checkValidity()

        #self.solve()
        np.set_printoptions(formatter={'float':'{:0.5f}'.format})
        #print(f"solution path:")
        #[print(f"RealVectorState {v}") for v in path_data.path_list]

    def solve(self):
        solved = self.ss.solve(self.planning_time)
        if solved and self.ss.haveExactSolutionPath():
            print("Found exact solution:")
            self.ss.simplifySolution(self.simplification_time)
        else:
            print("No exact solution found")

        data = ob.PlannerData(self.si)
        self.ss.getPlannerData(data)

        print(f"solution path:")

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