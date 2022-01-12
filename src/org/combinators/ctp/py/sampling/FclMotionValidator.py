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

collider_size = 1


class FclMotionValidator(ob.MotionValidator):
    def checkMotion(self, *args, **kwargs):
        #print("checking motion")
        s1 = args[0]
        s2 = args[1]
        #print("checking motion2")

        valid_motion, min_val = self.check_free_line_3d(
            [s1.getX(), s1.getY(), s1.getZ()],
            [s2.getX(), s2.getY(), s2.getZ()])

        if args.__len__() == 3:
            min_val2 = min_val - 0.01 if min_val - 0.01 >= 0.0 else 0.0
            min_val2 = min_val
            x = s1.getX() + min_val2 * (s2.getX() - s1.getX())
            y = s1.getY() + min_val2 * (s2.getY() - s1.getY())
            z = s1.getZ() + min_val2 * (s2.getZ() - s1.getZ())
            args[2].first.setX(x)
            args[2].first.setY(y)
            args[2].first.setZ(z)
            args[2].second = min_val2

        return valid_motion


    def motionIsValidForSceneObject(self, t_start,t_end, obj):
        #print("motionisvalid for")
        req_plain = fcl.ContinuousCollisionRequest()
        req_plain.gjk_solver_type = fcl.GJKSolverType.GST_LIBCCD
        req_plain.num_max_iterations = 10000
        req_plain.toc_err = 0.001
        req_plain.ccd_motion_type = fcl.CCDMotionType.CCDM_TRANS
        req_plain.ccd_solver_type = fcl.CCDSolverType.CCDC_NAIVE

        ccr1 = fcl.ContinuousCollisionResult()
        #print("calling continuousCollide1")

        fcl_obj, fcl_transform = obj

        ret1 = fcl.continuousCollide(
            fcl.CollisionObject(fcl.Box(collider_size, collider_size, collider_size), t_start), t_end,
            fcl.CollisionObject(fcl_obj), fcl_transform,
            req_plain,
            ccr1)

        if ret1 == 1.0:
            req2 = fcl.ContinuousCollisionRequest()
            req2.gjk_solver_type = fcl.GJKSolverType.GST_INDEP
            req2.num_max_iterations = 20000
            req2.toc_err = 0.001
            #req2.ccd_motion_type = fcl.CCDMotionType.CCDM_TRANS
            req2.ccd_motion_type = fcl.CCDMotionType.CCDM_SCREW
            req2.ccd_solver_type = fcl.CCDSolverType.CCDC_NAIVE
            ccr2 = fcl.ContinuousCollisionResult()

            print("calling continuousCollide2")

            ret2 = fcl.continuousCollide(
                fcl.CollisionObject(fcl.Box(collider_size, collider_size, collider_size), t_start), t_end,
                fcl.CollisionObject(fcl_obj), fcl_transform,
                req2,
                ccr2)

            if ret2 != 1.0:
                print(f"Warn Different result: {ret1}, {ret2}. Tocs: {ccr1.time_of_contact}, {ccr2.time_of_contact}")
                ret1 = ret2

        if ret1 == 1.0 and ccr1.is_collide:
            print(f"WARN 001: Inconsistent CCD results. result2.is_collide: {ccr1.is_collide}, ret2 == 1.0: {ret1 == 1.0}")
            print(f"WARN 002: Additional Info: result2.time_of_contact: {ccr1.time_of_contact}, ret2: {ret1}")
        if ccr1.time_of_contact != ret1:
            print(f"WARN 003: Inconsistent CCD results. result2.time_of_contact: {ccr1.time_of_contact}, ret2: {ret1}")
        return ret1


    def check_free_line_3d(self, *args, **kwargs):
        #print("checkFreeLine")
        t1 = args[0]
        t2 = args[1]
        t_1 = fcl.Transform(t1)
        t_2 = fcl.Transform(t2)

        results = []
        for o in fcl_scene_data.fcl_objs:
            results.append(self.motionIsValidForSceneObject(t_1, t_2, o))

        #print(results)
        min_val = min(results)

        #print("checkFreeLine2")

        # if not 1.0 - min_val < 0.01:
        #     if len(args) == 3:
        #         print(
        #             f"{args[0]}{args[1]}[{args[2].first.getX()}, {args[2].first.getY()}, {args[2].first.getZ()}[{args[2].second}[Result: False")
        #     else:
        #         print(f"{args[0]}{args[1]}[Result: False")

        # if args[0][0] == 30.0 and args[1][0] == -30.0:
        #     print(
        #         f"{args[0]}{args[1]}, Result: {1.0 - min_val < 0.01, min_val}")
        # print("checkFreeLine3")

        return 1.0 - min_val < 0.01, min_val
