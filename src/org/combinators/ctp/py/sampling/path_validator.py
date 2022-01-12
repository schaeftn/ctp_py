import pybullet_data

import numpy as np
import fcl

from org.combinators.ctp.py.sampling.gen import fcl_scene_data
from org.combinators.ctp.py.sampling.gen.path_data import path_list

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

import pybullet as p
import pybullet_utils.bullet_client as bc

max_distance = 1.0

def path_is_valid():
    last_state = []
    valid = True
    for state in path_list:
        if(last_state and valid):
            seg_dist = np.sum(np.square(state - last_state))
            number_steps = round(seg_dist / max_distance, 0)
            step_size = seg_dist / number_steps
            for i in range(0, number_steps):
                n_state = [f + i * (s - f) for f, s in zip(last_state, state)]
                valid = valid and state_is_valid(n_state)
        last_state = state
    return valid


def state_is_valid(r7):
    r3_state = [r7[0], r7[1], r7[2]]
    quaternion_data = [r7[3], r7[4], r7[5], r7[6]]
    fcl_transform = fcl.Transform(np.array(quaternion_data), np.array(r3_state))
    if hasattr(fcl_scene_data, 'robot_mesh'):
        robot_collision_object = fcl.CollisionObject(fcl_scene_data.robot_mesh, fcl_transform)
    else:
        robot_collision_object = fcl.CollisionObject(fcl.Sphere(1.0), fcl_transform)

    for (env_co, transform) in fcl_scene_data.fcl_objs:
        req = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        ret = fcl.collide(robot_collision_object, fcl.CollisionObject(env_co, transform), req, result)

        if result.is_collision:
            return False
    return True


if __name__ == '__main__':
    print("starting path validation")

    if path_is_valid():
        print("Path is valid")
    else:
        print("Path is not valid")
