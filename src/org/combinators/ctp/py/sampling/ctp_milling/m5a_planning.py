import sys

sys.path.append("/home/tristan/projects/ctp_milling_py/src")

# noinspection PyUnresolvedReferences

import logging
import logging.config

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


from  m5a_sampling import M5aValidityChecker, M5aValidStateSampler, WeightedEuclideanCostObjective

import numpy as np
import os

planner_id = 0
sampler = 0

current_dir = os.path.dirname(__file__)
conf_location = current_dir[:current_dir.index("/src")] + os.sep + "logging.conf"
logging.config.fileConfig(conf_location)
logger = logging.getLogger('planning')

class DiscreteMotionValidatonWrapper(ob.DiscreteMotionValidator):
    def __init__(self, si):
        super().__init__(si)
        self.mv2 = ob.DiscreteMotionValidator(si)

    def checkMotion(self, *args, **kwargs):
        s1 = args[0]
        s2 = args[1]
        s3b = []

        if args.__len__() == 3:
            s3b = [args[2].first[0], args[2].first[1], args[2].first[2]]

        valid_motion = self.mv2.checkMotion(*args, **kwargs)

        if args.__len__() == 2 and valid_motion:
            logger.debug("2 args")
            logger.debug(f"s1: {s1[0], s1[1], s1[2]}, s2: {s2[0], s2[1], s2[2]}, valid: {valid_motion}")

        if args.__len__() == 3:
            logger.debug("3 args")
            logger.debug(
                f"s1: {s1[0], s1[1], s1[2]},"
                f"s2: {s2[0], s2[1], s2[2]},"
                f"s3 before: {s3b[0], s3b[1], s3b[2]},"
                f"s3 after: {args[2].first[0], args[2].first[1], args[2].first[2]},"
                f"valid: {valid_motion}")

        return valid_motion


# check https://github.com/kucars/laser_collision_detection/blob/1b78fe5a95584d135809b1448d33675bb8fee250/src/laser_obstacle_detect.cpp#L252
# data format, resolution, ansonsten check api fcl...
# 3d planning for fixed alpha beta
class plan:
    def __init__(self, s, start, end, alpha, beta, d_vector):
        self.samplerIndex = s

        # construct the state space we are planning in
        # x,y,z,a,b
        space = ob.RealVectorStateSpace(5)

        # set the bounds
        bounds = ob.RealVectorBounds(5)
        bounds.setLow(-50)
        bounds.setHigh(50)
        space.setBounds(bounds)

        # create a start state
        s = ob.State(space)
        # todo methode array to state
        s[0] = start[0]
        s[1] = start[1]
        s[2] = start[2]
        s[3] = 0.0
        s[4] = 0.0

        # create a goal state
        goal = ob.State(space)
        goal[0] = end[0]
        goal[1] = end[1]
        goal[2] = end[2]
        goal[3] = 0.0
        goal[4] = 0.0

        def foo(space_information):
            return ob.ObstacleBasedValidStateSampler(space_information)

        def vss(space_information):
            return M5aValidStateSampler(space_information, start, end, alpha, beta, displacement_vector=d_vector)

        si = ob.SpaceInformation(space)
        validityChecker = M5aValidityChecker(si, start, end, alpha, beta, d_vector)
        si.setStateValidityChecker(validityChecker)
        s_alloc = ob.ValidStateSamplerAllocator(vss)
        si.setValidStateSamplerAllocator(s_alloc)
        si.setStateValidityCheckingResolution(0.0005)
        si.setMotionValidator(DiscreteMotionValidatonWrapper(si))
        #si.setMotionValidator(ob.DiscreteMotionValidator(si))

        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(s, goal)
        weights = np.array([1.0, 1.0, 1.0, 100.0, 100.0])
        objective = WeightedEuclideanCostObjective(si, start, end, weights)
        pdef.setOptimizationObjective(objective)

        # self.si.setStateValidityChecker(ob.StateValidityCheckerFn(vss(self.si).isStateValid))
        # self.si.setStateValidityChecker(ob.StateValidityCheckerFn(partial(isValid, np.array(start), np.array(end), alpha, beta)))
        #self.si.setMotionValidator(MyDiscreteMotionValidator(self.si))

        planner = og.PRMstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        planner.checkValidity()

        solved = planner.solve(5.0)
        if solved and pdef.hasExactSolution():
            logger.info("Found solution:")
            logger.info(f"{planner.getName()} found solution of path length {pdef.getSolutionPath().length()} with an "
                  f"optimization objective value"
                  f" of {pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()}")

            # print the path to screen
            # logger.debug(data.printGraphML())
            data = pdef.getSolutionPath()
            logger.info(f"solution path: {data}")
            logger.debug(f"simplify")

            simp = og.PathSimplifier(si, pdef.getGoal(), objective)
            logger.debug(f"simplify")
            simp.simplify(data, 5.0)
            logger.debug(f"simplify")

            # logger.debug("Found simplified solution:")
            # logger.debug(f"{planner.getName()} found solution of path length {pdef.getSolutionPath().length()} with an "
            #       f"optimization objective value"
            #       f" of {pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()}")
            #
            # data2 = pdef.getSolutionPath()
            # logger.debug(f"solution path: {data2}")
            #
            #
            # simp.simplifyMax(pdef.getSolutionPath())
            # logger.debug("Found max simplified solution:")
            # logger.debug(f"{planner.getName()} found solution of path length {pdef.getSolutionPath().length()} with an "
            #       f"optimization objective value"
            #       f" of {pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()}")
            #
            # data2 = pdef.getSolutionPath()
            # logger.debug(f"solution path: {data2}")

            logger.info("Found bSpline solution:")
            logger.info(f"{planner.getName()} found solution of path length {pdef.getSolutionPath().length()} with an "
                  f"optimization objective value"
                  f" of {pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()}")

            data2 = pdef.getSolutionPath()
            logger.info(f"solution path: {data2}")
            simp.smoothBSpline(pdef.getSolutionPath())

            # useGraphTool(data)
        else:
            logger.info("No solution found")


if __name__ == '__main__':
    s = [0.0, 0.0, 0.0]  # start point
    g = [5.0, 0.0, 0.0]  # end point
    d_vector = [0.0, 1.2, 0.0]  ##
    p = plan(planner_id, s, g, 0.0, 0.0, d_vector)  # uniform sampling
