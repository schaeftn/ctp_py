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
        
        space = ob.SE3StateSpace()

        # set the bounds
                # create R^3 Bounds
        bounds = ob.RealVectorBounds(3) #
        bounds.setLow(0, -508.88)
        bounds.setHigh(0, 319.62)
        bounds.setLow(1, -230.13)
        bounds.setHigh(1, 531.87)
        bounds.setLow(2, -123.75)
        bounds.setHigh(2, 101.0)
        bounds.check()
        space.setBounds(bounds)
        

        # create a start state
        start = ob.State(space)
        startRef = start()
        startRef.setX(-14.96)
        startRef.setY(-48.62)
        startRef.setZ(70.57)
        startRef.rotation().setAxisAngle(1.0,0,0,1.570796326794)
        # startRef.rotation().setIdentity()  # start[0] = 0
        # start[1] = 0
        # start[2] = 0

        # create a goal state
        goal = ob.State(space)
        goalRef = goal()
        goalRef.setX(187.58)
        goalRef.setY(-48.62)
        goalRef.setZ(70.57)
        goalRef.rotation().setAxisAngle(1.0,0,0,1.570796326794)
        # goalRef.rotation().setIdentity()

        space.setStateSamplerAllocator(ob.StateSamplerAllocator(ob.CompoundStateSampler))
        self.si = ob.SpaceInformation(space)
        self.si.setMotionValidator(FclMotionValidator(self.si))
        self.si.setStateValidityChecker(FclStateValidator(self.si))
        self.si.setStateValidityCheckingResolution(0.01)
        


        self.ss = og.SimpleSetup(self.si)
        p = og.RRTConnect(self.si)
        self.ss.setPlanner(p)
        self.ss.setStartAndGoalStates(start, goal)
        self.ss.setup()
        self.ss.getPlanner().checkValidity()

        self.solve()
        np.set_printoptions(formatter={'float':'{:0.5f}'.format})
        #print(f"solution path:")
        #[print(f"RealVectorState {v}") for v in path_data.path_list]

    def solve(self):
        solved = self.ss.solve(self.planning_time)
        if solved and self.ss.haveExactSolutionPath():
            print("Found exact solution:")
            print(f"Simplification time: {self.simplification_time}")
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