import sys

import math
import numpy as np
from ompl import util as ou
from ompl import base as ob

import logging
import logging.config
import os

chatter_max = 50.0  # micrometer

current_dir = os.path.dirname(__file__)
conf_location = current_dir[:current_dir.index("/src")] + os.sep + "logging.conf"
logging.config.fileConfig(conf_location)
logger = logging.getLogger('sampling')


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


class M5aValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si, start_3d, end_3d, alpha, beta, displacement_vector):
        super(M5aValidStateSampler, self).__init__(si)
        self.name_ = "my sampler"
        self.rng_ = ou.RNG()
        self.start_ = np.array(start_3d)
        self.end_ = np.array(end_3d)
        self.alpha_ = alpha
        self.beta_ = beta
        self.displacement_vector_ = np.array(displacement_vector)
        logger.debug(f"displacement vector: {self.displacement_vector_}")
        logger.debug(f"start: {self.start_}, end: {self.end_}")
        logger.debug(f"angles. alpha: {self.alpha_}, beta: {self.beta_}")

    # in welche vectorrichtung soll gesucht werden? Übersetzung in ein 2D Planungsproblem, p1: linarparameter, fixed other, p2:
    # find valid state and set composite state accordingly
    def sample(self, *args, **kwargs):
        r = self.rng_.uniformReal(0, 1)
        alpha_sample = self.rng_.gaussian(self.alpha_, 0.7) # about 80%+ are in (-1,1)
        beta_sample = self.rng_.gaussian(self.beta_, 0.7) # about 80%+ are in (-1,1)

        sample_point = (1 - r) * self.start_ + r * self.end_
        point = sample_point + self.displacement_vector_ * chatter_fct(sample_point[0], sample_point[1],
                                                                       sample_point[2], alpha_sample, beta_sample)

        # if not close_to(point[1], 0.6) and not close_to(point[1], 40.6):
        #     logger.debug("asd")

        # logger.debug(f"r: {r}")
        # logger.debug(f"point x: {sample_point[0]}, point y: {sample_point[1]}, point z: {sample_point[2]}")
        # logger.debug(f"valid sampled point x: {point[0]}, point y: {point[1]}, point z: {point[2]}")
        # #logger.debug(f"pointisValid: {self.isStateValid(state)}, r: {r}")
        #
        # logger.debug(f"sample_before: {args[0][0], args[0][1], args[0][2]}, sample_after: {point[0], point[1], point[2]}")
        args[0][0] = point[0]
        args[0][1] = point[1]
        args[0][2] = point[2]
        args[0][3] = alpha_sample
        args[0][4] = beta_sample
        logger.debug(f"{args[0][0], args[0][1], args[0][2], args[0][3], args[0][4]}")

        return True

    def sampleNear(self, *args, **kwargs):
        logger.debug(f"sampleNear called")
        return False


def line_point_vector(a, b):
    return a * np.dot(b, a) / np.dot(a, a)


class M5aValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle
    def __init__(self, si, start_3d, end_3d, alpha, beta, displacement_vector):
        super().__init__(si)
        self.start_ = np.array(start_3d)
        self.end_ = np.array(end_3d)
        self.alpha_ = alpha
        self.beta_ = beta
        self.displacement_vector_ = np.array(displacement_vector)

    def isValid(self,  *args, **kwargs):
        state = args[0]
        x = state[0]
        y = state[1]
        z = state[2]
        alpha = state[3]
        beta = state[4]

        # if 0.5 < x < 1.0 and not s1_valid(y):
        #     logger.debug(f"s1_valid(y): {s1_valid(y)}")
        #     logger.debug(f"close to {y}, {60.0}: {close_to(y, 60.0)}")
        #     logger.debug(f"care, isValid called for (maybe) intepolated value: {x, y, z}")

        if not self.geo_is_valid(x, y, z):
            # if 0.5 < x < 1.0:
            #     logger.debug(f"geo state invalid: {x,y,z}")
            return False

        chatter = chatter_fct(x, y, z, alpha, beta)
        if not chatter_is_valid(chatter):
            # logger.debug(f"chatter invalid for point [{x, y, z}]")
            # if 0.5 < x < 1.0:
            #     logger.debug(f"chatter state invalid: {x, y, z}")
            return False

        if self.clearance(state) < 0.0:
            # logger.debug(f"chatter violation for point [{state[0], state[1], state[2]}]")
            # if 0.5 < state[0] < 1.0:
            #     logger.debug(f"chatter state violation: {state[0], state[1], state[2]}")
            return False

        # if 0.5 < state[0] < 1.0 and not s1_valid(state[1]):
        #     logger.debug(f"should not happen iii state valid: {state[0], state[1], state[2]}")

        return True

    # clearance distance - chatter
    def clearance(self, *args, **kwargs):
        state = args[0]
        x = state[0]
        y = state[1]
        z = state[2]
        alpha = state[3]
        beta = state[4]

        if alpha > 10.0:
             return 0.05

        return clearance_s(x, y, z, self.start_, self.end_, alpha, beta)

        # line_point = self.start_ + line_point_vector(self.end_ - self.start_, np.array([x, y, z]) - self.start_)
        # chatter_value = chatter_fct(line_point[0], line_point[1], line_point[2], self.alpha_, self.beta_)
        # distance = math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(line_point, np.array([x, y, z]))]))
        # # logger.debug(f"point: {x, y, z},"
        # #       f" line_point: {line_point[0], line_point[1], line_point[2]},"
        # #       f" distance: {distance}, c_val: {chatter_value}")
        #
        # return distance - chatter_value

    def geo_is_valid(self, x, y, z):
        start_point_v = np.array([x, y, z]) - self.start_
        plane_vector = np.cross(self.end_ - self.start_, self.displacement_vector_)
        plane_vector = plane_vector / np.sqrt(np.dot(plane_vector, plane_vector))
        # line_point = self.start_ + line_point_vector(self.end_ - self.start_, np.array([x, y, z]) - self.start_)
        # distance = math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(line_point, np.array([x, y, z]))]))

        distance = np.abs(np.dot(plane_vector, start_point_v))

        # line_point + distance * self.displacement_vector_
        # distance2 =  math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(rec_p, np.array([x, y, z]))]))

        return distance < 0.0000001


def s1_valid(s):
    close_to(s, 60.0) or close_to(s, 40.0) or (40.0 < s < 60.0)


def close_to(v, v2):
    return -0.000001 < (v - v2) < 0.000001


def clearance_s(x, y, z, start_3d, end_3d, alpha, beta):
    line_point = start_3d + line_point_vector(end_3d - start_3d, np.array([x, y, z]) - start_3d)
    chatter_value = chatter_fct(line_point[0], line_point[1], line_point[2], alpha, beta)
    distance = math.sqrt(sum([(xi - yi) ** 2 for xi, yi in zip(line_point, np.array([x, y, z]))]))
    # logger.debug(f"point: {x, y, z},"
    #       f" line_point: {line_point[0], line_point[1], line_point[2]},"
    #       f" distance: {distance}, c_val: {chatter_value}")
    #logger.debug(f"clearance: {distance - chatter_value}")
    return distance - chatter_value


# np array to support element wise addition, substraction, multiplication
def state_to_array(s):
    return np.array([s[0], s[1], s[2]])


def chatter_fct(x, y, z, alpha, beta):
    if 0.5 < x < 1.0:
        return 40.0
    return 0.0


def chatter_is_valid(x):
    return x < chatter_max


def main():
    logger.debug("vss_5a_geometry_main")


if __name__ == '__main__':
    main()