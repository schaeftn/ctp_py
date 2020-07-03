import numpy as np
import fcl

from org.combinators.ctp.py.sampling.gen import fcl_scene_data

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys


class FclStateValidator(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)

    def isValid(self, *args, **kwargs):
        ompl_state = args[0]
        r3_state = [ompl_state.getX(), ompl_state.getY(), ompl_state.getZ()]
        quaternion_data = [ompl_state.rotation().w, ompl_state.rotation().x, ompl_state.rotation().y,
                           ompl_state.rotation().z]
        # print(f"isValid for: {r3_state}, {quaternion_data}")
        fcl_transform = fcl.Transform(np.array(quaternion_data), np.array(r3_state))
        if hasattr(fcl_scene_data, 'robot_mesh'):
            robot_collision_object = fcl.CollisionObject(fcl_scene_data.robot_mesh, fcl_transform)
        else:
            # print(f"defaulting to unit sphere")
            robot_collision_object = fcl.CollisionObject(fcl.Sphere(1.0), fcl_transform)


        for (env_co, transform) in fcl_scene_data.fcl_objs:
            req = fcl.CollisionRequest()
            result = fcl.CollisionResult()
            ret = fcl.collide(robot_collision_object, fcl.CollisionObject(env_co, transform), req, result)

            if result.is_collision:
                return False

        return True
