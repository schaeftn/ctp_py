from functools import partial

import numpy as np
import fcl

from CGAL.CGAL_Kernel import intersection, Polygon_2, Point_2, do_intersect, ON_BOUNDED_SIDE, ON_UNBOUNDED_SIDE
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

ptlst = []
ptlst.append(Point_2(0, 0))
ptlst.append(Point_2(-1, 1))
ptlst.append(Point_2(1, 1))

ptlst2 = [Point_2(0, 0),Point_2(0, 0),Point_2(0, 0),Point_2(0, 0)]
ptlst2.append(Point_2(0, 0))
ptlst2.append(Point_2(1, 0))
ptlst2.append(Point_2(1, 1))
ptlst2.append(Point_2(0, 1))
# hoursglass => area = 0.0

obstacle0=Polygon_2(ptlst)
obstacle1=Polygon_2(ptlst2)
scene_objects = [obstacle0,obstacle1]

def motionIsValidForSceneObject(t_start, t_end, obj):
    # TODO Build Line, intersect with all objects
    return True

def check_free_line_3d(*args, **kwargs):
    return True, 0.0

def is_valid(*args, **kwargs): # args[0] is state
    #Todo intersect Test with all objects
    p = Point_2(args[0][0],args[0][1])
    lst = [obj.bounded_side(p) for obj in scene_objects]
    return ON_UNBOUNDED_SIDE in lst

def cgal_precision_test():
    print('fooÄ')
    print(is_valid([0.0, 1.000001]))
    print(is_valid([0.0, 1.00000001]))
    print(is_valid([0.0, 1.0000000001]))
    print(is_valid([0.0, 1.0000000000001]))
    print(is_valid([0.0, 1.00000000000000001]))
    print(is_valid([0.0, 1.00000000000000000000001]))
    print(is_valid([0.0, 1.0000000000000000000000000001]))

#Clockwise: Negative
def area_test():
    print(f"Area: {scene_objects[0].area()}")
    print(f"Orientation: {scene_objects[0].orientation()}")
    scene_objects[0].reverse_orientation()
    print(f"Area: {scene_objects[0].area()}")
    print(f"Orientation: {scene_objects[0].orientation()}")

    print(f"o1 Area: {scene_objects[1].area()}")
    print(f"o1 Orientation: {scene_objects[1].orientation()}")


def main():
    area_test()

if __name__ == '__main__':
    print("Running simple cgal test")
    main()