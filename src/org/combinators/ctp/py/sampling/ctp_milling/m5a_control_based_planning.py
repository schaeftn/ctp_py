import sys

from org.combinators.ctp.py.ctp_milling.m5a_sampling import WeightedEuclideanCostObjective

import numpy as np

sys.path.append("/home/tristan/projects/ctp_sampling_py/src")

import logging
import logging.config
import os
import functools

current_dir = os.path.dirname(__file__)
conf_location = current_dir[:current_dir.index("/src")] + os.sep + "logging.conf"
logging.config.fileConfig(conf_location)
logger = logging.getLogger('planning')

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import control as oc
    from ompl import base as ob
    from ompl import geometric as og

planner_id = 0
sampler = 0


# class M5aCost(ob.StateCostIntegralObjective):
#     def __init__(self, p_object, *args, **kwargs):
#         super().__init__(p_object, *args, **kwargs)
#
#     def stateCost(self, *args, **kwargs):
#         return 100.0 - args[0][1]

class M5a_ControlSampler(oc.ControlSampler):
    def __init__(self, control_space):
        super(M5a_ControlSampler, self).__init__(control_space)
        self.rng_ = ou.RNG()
        self.space_ = control_space

    def sample(self, *args, **kvargs):
        logger.debug(f"controlSampler  callsd")
        raise NotImplementedError("fuuuuu")

    # sampleNext (Control *control, const Control *previous, const base::State *state)
    def sampleNext(self, *args, **kwargs):
        logger.debug(f"sampleNext. control_sample before: {args[0]}")
        # self.sample(*args)
        args[0][0] = self.get_bounded_val(args[1][0] + self.rng_.gaussian(0.0, 0.05), 0)  # ddx/dt < 0.2
        if args[2][1] < 0.8:
            args[0][1] = self.get_bounded_val(self.rng_.gaussian(5.0, 2.0), 1)  # ddy/dt < 0.2
        else:
            args[0][1] = self.get_bounded_val(self.rng_.gaussian(-5.0, 2.0), 1)  # ddy/dt < 0.2

        args[0][2] = self.get_bounded_val(args[1][2] + self.rng_.gaussian(0.0, 0.05), 2)  # ddz/dt < 0.2
        args[0][3] = self.get_bounded_val(args[1][3] + self.rng_.gaussian(0.0, 0.2), 3)  # ddz/dt < 0.2
        args[0][4] = self.get_bounded_val(args[1][4] + self.rng_.gaussian(0.0, 0.2), 4)  # ddz/dt < 0.2
        logger.debug(f"sampleNext. control_sample after: {args[0]}")

    def get_bounded_val(self, val, dim):
        bounds = self.space_.getBounds()
        low = bounds.low[dim]
        high = bounds.high[dim]
        if low <= val <= high:
            return val
        elif val < low:
            return low
        return high


def propagate(state_in, control, delta_t, state_out):
    s = f"\r\nstate_in:  {state_in[0], state_in[1], state_in[2], state_in[3], state_in[4]}\r\n"
    s += f"control:   {control[0], control[1], control[2], control[3], control[4]}\r\n"
    logger.debug(f"propagate called. delta_t: {delta_t}")
    state_out[0] = state_in[0] + control[0] * delta_t
    state_out[1] = state_in[1] + control[1] * delta_t
    state_out[2] = state_in[2] + control[2] * delta_t
    state_out[3] = state_in[3] + control[3] * delta_t
    state_out[4] = state_in[4] + control[4] * delta_t

    s += f"state_out: {state_out[0], state_out[1], state_out[2], state_out[3], state_out[4]}"
    logger.debug(f"{s}")


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




class plan:
    def __init__(self, s):

        self.samplerIndex = s
        # construct the state space we are planning in

        space = ob.RealVectorStateSpace(0)
        space.addDimension("x", -100.0, 100.0)
        space.addDimension("y", -100.0, 100.0)
        space.addDimension("z", -100.0, 100.0)
        space.addDimension("alpha", -1.0, 1.0)
        space.addDimension("beta", -1.0, 1.0)
        space.setup()

        # create a start state
        start = ob.State(space)
        start[0] = 0.0
        start[1] = 0.0
        start[2] = 0.0
        start[3] = 0.0
        start[4] = 0.0

        # create a goal state
        goal = ob.State(space)
        goal[0] = 1.0
        goal[1] = 4.0
        goal[2] = 4.0
        goal[3] = 0.0
        goal[4] = 0.0

        # create a control space
        cspace = oc.RealVectorControlSpace(space, 5)

        # set the bounds for the control space
        cbounds = ob.RealVectorBounds(5)
        cbounds.setLow(0, -.1)
        cbounds.setHigh(0, .1) # max 0.1 dx/dt
        cbounds.setLow(1, -5.0)
        cbounds.setHigh(1, 5.0) # max 0.1 dy/dt
        cbounds.setLow(2, -.1)
        cbounds.setHigh(2, .1) # max 0.1 dz/dt
        cbounds.setLow(3, 0.0)
        cbounds.setHigh(3, .05) # max 0.05 dalpha/dt
        cbounds.setLow(4, 0.0)
        cbounds.setHigh(4, .05) # max 0.05 dbeta/dt
        cbounds.check()
        cspace.setBounds(cbounds)
        cspace.setup()

        si = oc.SpaceInformation(space, cspace)
        #si.setMotionValidator(ob.DiscreteMotionValidator(si))
        #ss = oc.SimpleSetup(cspace)

        si.setStateValidityChecker(ob.StateValidityCheckerFn(functools.partial(isStateValid, space.getBounds())))
        si.setStatePropagator(oc.StatePropagatorFn(propagate))
        asd = DiscreteMotionValidatonWrapper(si)
        si.setMotionValidator(asd)
        si.setStateValidityCheckingResolution(0.0005)
        #si.setStateSampler()
        si.setup()

        # self.si.setStatePropagatorFn(oc.StatePropagatorFn(propagate))
        def alloc_c_sampler(param_c_space):
            return M5a_ControlSampler(param_c_space)

        cspace.setControlSamplerAllocator(oc.ControlSamplerAllocator(alloc_c_sampler))
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
        # weights = np.array([1.0, 1.0, 1.0, 100.0, 100.0])
        # objective = WeightedEuclideanCostObjective(si, start, goal, 0.0, 0.0, [0.0,1.0,0.0], weights)
        # pdef.setOptimizationObjective(objective)

        logger.debug(f"stepsize: {si.setMinMaxControlDuration(1, 10)}")
        logger.debug(f"stepsize: {si.setPropagationStepSize(1.0)}")
        logger.debug(f"stepsize: {si.getPropagationStepSize()}")
        #ss.setOptimizationObjective(M5aCost(self.si))
        #ss.setup()
        #ss.getPlanner().checkValidity()
        #ss.getPlanner().checkValidity()
        planner = oc.KPIECE1(si)
        planner.setProblemDefinition(pdef)
       # planner.setIntermediateStates(True)
        #cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

        planner.checkValidity()
        planner.setup()
        solved = planner.solve(5.0)

        logger.debug(f"Goal bias: {planner.getGoalBias()}")
        if solved:
            solution_is_approx = pdef.hasApproximateSolution()
            logger.info(f"""Found {"approximate" if solution_is_approx else "exact" } solution:""")
            logger.info(f"{planner.getName()} found solution of path length {pdef.getSolutionPath().length()}")
            pc = oc.PathControl(si)
            logger.info(f"pc.getStates(): {pc.getStates()}")
            logger.info(f"pdef.getSolutionPath().getStates(): {pdef.getSolutionPath().getStates()}")
            logger.info(f"pdef.getSolutionPath().getControls(): {pdef.getSolutionPath().getControls()}")
            # print the path to screen
            # logger.debug(data.printGraphML())
            data = pdef.getSolutionPath()
            logger.info(f"solution path: {data}")
            # control based path smoothing?
        else:
            logger.info("No solution found")


    # This function is needed, even when we can write a sampler like the one
    # above, because we need to check path segments for validity
def isStateValid(bounds, *args, **kvargs):
    valid_list = [low <= val <= high for low, high, val in zip(bounds.low, bounds.high, args[0])]
    if False in valid_list:
        return False
    if 0.4 < args[0][0] < 0.8:  # in interval x (0.4, 0.5), y needs to be > 10
        if args[0][1] < 10.0:
            logger.debug(f"returned false for values {args[0][0], args[0][1]}")
            return False
        else:
            logger.debug(f"returned true for values {args[0][0], args[0][1]}")
            return True
    return True


if __name__ == '__main__':
    logger.info("Main, starting")
    p = plan(planner_id)  # uniform sampling
