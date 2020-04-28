from enum import Enum
from sympy.geometry.line import Line, Segment
from sympy.geometry.util import *

import sys

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene import Scene, Scene2D
from org.combinators.ctp.py.celldecomposition.scene_object import *


class VerticalLineDirection(Enum):
    up = 1
    down = 2


def sort_second(val):
    return val[0]


def get_ordered_vertices(scene: Scene):
    v = []
    for so in scene.scene_objects:
        v.extend(so.vertices)
    v.sort(key=sort_second)
    return np.array(v)


def vcd_2d(scene_objects, scene_size):
    scene = Scene2D(scene_objects, scene_size)
    vertices = get_ordered_vertices(scene)
    new_lines = []

    x_map = {x[0] for x in vertices}
    linemap = [Segment(Point(x, -scene_size[1] / 2), Point(x, scene_size[1] / 2)) for x in x_map]

    ints = find_all_intersections(linemap, scene)
    all_intersections = prune_is_list(ints)
    for current_obj in scene.objects_sympy:
        new_lines.extend(check_collision_2d(current_obj, scene, all_intersections, VerticalLineDirection.up))
        new_lines.extend(check_collision_2d(current_obj, scene, all_intersections, VerticalLineDirection.down))

    print("""{\n\t"lines" : """)
    print(f"""{np.array2string(np.array(new_lines, dtype=float), precision=6, separator=", ", floatmode="fixed")}\n}}""")

    for l in new_lines:
        scene.scene_objects.append(SceneObjectLine2d(l[0], l[1]))
    scene.export_model()

def check_collision_2d(o: Polygon, s: Scene2D, all_intersections, vld: VerticalLineDirection):
    vld_up = vld == VerticalLineDirection.up
    boundary_top = s.get_top_y_boundary()
    boundary_bottom = s.get_bottom_y_boundary()
    new_lines = []
    for p in o.vertices:
        c_inter = filter(lambda x: x[0] == p.x, all_intersections)
        first = find_first([p.x, p.y], c_inter, vld)

        if len(intersection(first, o)) == 0:
            new_lines.append([[p.x, p.y], first])
        else:
            distance = p.distance(Point(*first))
            if distance == 0:
                new_lines.append(
                    [[p.x, p.y], [p.x, boundary_top if vld_up else boundary_bottom]])
    return new_lines


def prune_is_list(l):
    new_list = []
    for i in l:
        if isinstance(i, Segment):
            new_list.extend([i.p1, i.p2])
        else:
            new_list.append(i)
    return np.array(new_list)


def find_first(vertex, intersections, direction: VerticalLineDirection):
    y_coord = vertex[1]
    up_dir = direction == VerticalLineDirection.up

    lst = [sympy_geo_object for sympy_geo_object in intersections if
           (sympy_geo_object[1] - y_coord > 0.01 if up_dir else sympy_geo_object[1] - y_coord < -0.01)]

    lst2 = [[p[0], p[1]] for p in lst]
    vertex_first = min(lst2, default=vertex) if up_dir else max(lst2, default=vertex)
    return vertex_first


def find_all_intersections(lines, s: Scene2D):
    #print("intersecting")
    #print(*s.objects_sympy)
    is_list = intersection(*s.objects_sympy, *lines, pairwise=True)
    #print(is_list)
    #print("Done Intersecting")
    return [i for i in is_list if i]
