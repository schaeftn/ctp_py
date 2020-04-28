import sys

from org.combinators.ctp.py.sampling.FclStateValidator import FclStateValidator

sys.path.append("/home/tristan/projects/ctp_sampling_py/src")

from org.combinators.ctp.py.sampling.FclMotionValidator import *

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

planner_id = 0
sampler = 0

#
# class SceneMotionValidator(ob.MotionValidator):
#     def __init__(self, si, fcl_scene):
#         super(SceneMotionValidator, self).__init__(si)
#         self.scene = fcl_scene
#         self.state = ob.State(ob.SE3StateSpace())
#
#     def checkMotion(self, *args, **kwargs):
#         s1 = args[0]
#         s2 = args[1]
#         valid_motion = False
#
#         if args.__len__() == 2:
#             valid_motion, min_val = self.scene.check_free_line_3d(
#                 [s1.getX(), s1.getY(), s1.getZ()],
#                 [s2.getX(), s2.getY(), s2.getZ()])
#
#         if args.__len__() == 3:
#             valid_motion, col_param = self.scene.check_free_line_3d(
#                 [s1.getX(), s1.getY(), s1.getZ()],
#                 [s2.getX(), s2.getY(), s2.getZ()])
#             min_val = col_param - 0.01 if col_param - 0.01 >= 0.0 else 0.0
#             x = s1.getX() + min_val * (s2.getX() - s1.getX())
#             y = s1.getY() + min_val * (s2.getY() - s1.getY())
#             z = s1.getZ() + min_val * (s2.getZ() - s1.getZ())
#             args[2].first.setX(x)
#             args[2].first.setY(y)
#             args[2].second = min_val
#
#         return valid_motion


# check https://github.com/kucars/laser_collision_detection/blob/1b78fe5a95584d135809b1448d33675bb8fee250/src/laser_obstacle_detect.cpp#L252
# data format, resolution, ansonsten check api fcl...
class plan:
    def __init__(self, s):
        self.samplerIndex = s
        # construct the state space we are planning in
        space = ob.SE3StateSpace()

        # set the bounds
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(-50)
        bounds.setHigh(50)
        space.setBounds(bounds)

        # create a start state
        start = ob.State(space)
        startRef = start()
        startRef.setX(30.0)
        startRef.setY(0.0)
        startRef.setZ(0.0)
        startRef.rotation().setIdentity()  # start[0] = 0
        # start[1] = 0
        # start[2] = 0

        # create a goal state
        goal = ob.State(space)
        goalRef = goal()
        goalRef.setX(-30.0)
        goalRef.setY(0.0)
        goalRef.setZ(0.0)
        goalRef.rotation().setAxisAngle(0.0, 0.0, 0.0, 0.0)



        self.si = ob.SpaceInformation(space)
        #self.si.setMotionValidator(ob.DiscreteMotionValidator(self.si))
        self.si.setMotionValidator(FclMotionValidator(self.si))
        self.si.setStateValidityChecker(FclStateValidator(self.si))
        self.si.setStateValidityCheckingResolution(0.0001)
        #self.si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(foo))

        self.ss = og.SimpleSetup(self.si)
        self.ss.setPlanner(og.LazyPRMstar(self.si))
        self.ss.setStartAndGoalStates(start, goal)
        self.ss.setup()
        self.ss.getPlanner().checkValidity()
        self.solve()


    def solve(self):
        solved = self.ss.solve(10.0)
        if solved and self.ss.haveExactSolutionPath():
            print("Found solution:")
            # print the path to screen
            data = ob.PlannerData(self.si)
            self.ss.getPlannerData(data)
            # print(data.printGraphML())
            if self.ss.haveSolutionPath():
                print(f"solution path: {self.ss.getSolutionPath()}")
            else:
                print(f"WARN: no solution path")


            #useGraphTool(data)
        else:
            print("No solution found")

    @staticmethod
    def getStateVector(s):
        return [s.getX(), s.getY(), s.getZ()]



if __name__ == '__main__':
    print("Using default uniform sampler:")
    p = plan(planner_id)  # uniform sampling