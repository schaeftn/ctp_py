import sys

from enum import Enum
from scipy.spatial.distance import euclidean
from sympy.geometry.line import Line, Segment
from sympy.geometry.util import *

import numpy as np

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene_object import *

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

class VerticalLineDirection(Enum):
    up = 1
    down = 2

# min_object distance = 0.05

class Vcd2D:
    def __init__(self, vertices, scene_objects, scene_objects_aabb, scene_size):
        self.vertices = vertices
        self.new_vertices = []
        self.scene_objects = scene_objects
        self.scene_objects_aabb = scene_objects_aabb
        self.lines = []  # int array
        self.scene_size = scene_size  # scene_size = [100.0, 100.0, 0.0]
        self.objects_sympy = [Polygon(*[self.vertices[v] for v in so]) for so in self.scene_objects]
        self.bottom_vertices = []
        self.top_vertices = []


    @property
    def vertices(self):
        return self._vertices

    @property
    def scene_objects(self):
        return self._scene_objects  # [[Int, int, int, int],[ints,int,int, int]]

    @property
    def scene_objects_aabb(self):
        return self._scene_objects_aabb  # [[minx,maxx], [miny,maxy]])]

    @property
    def scene_size(self):
        return self._scene_size

    @property
    def objects_sympy(self):
        return self._objects_sympy

    @property
    def bottom_vertices(self):
        return self._bottom_vertices

    @property
    def top_vertices(self):
        return self._top_vertices

    @vertices.setter
    def vertices(self, value):
        self._vertices = value

    @scene_objects.setter
    def scene_objects(self, value):
        self._scene_objects = value

    @scene_objects_aabb.setter
    def scene_objects_aabb(self, value):
        self._scene_objects_aabb = value

    @scene_size.setter
    def scene_size(self, value):
        self._scene_size = value

    @objects_sympy.setter
    def objects_sympy(self, value):
        self._objects_sympy = value

    @bottom_vertices.setter
    def bottom_vertices(self, value):
        self._bottom_vertices = value

    @top_vertices.setter
    def top_vertices(self, value):
        self._top_vertices = value

    def run(self):
        new_lines = []
        new_object_vertices = []
        for obj_id in range(len(self.scene_objects)):
            for v_id in self.scene_objects[obj_id]:
                minmax = self.find_first_max_min(v_id, obj_id)
                for p in minmax:
                    if p[0] == obj_id:
                        continue
                    if p[1] not in self.vertices:
                        self.vertices.append(p[1])
                        if p[0] >= 0:
                            new_object_vertices.append([p[0], self.vertices.index(p[1])])
                    if p[0] == -1:
                        self.top_vertices.append(self.vertices.index(p[1]))
                    if p[0] == -2:
                        self.bottom_vertices.append(self.vertices.index(p[1]))
                    new_lines.append([v_id, self.vertices.index(p[1])])
        for new_entry in new_object_vertices:
            self.scene_objects[new_entry[0]].append(new_entry[1])
        self.printJson(new_lines)

    def get_y_boundary(self):
        return self.scene_size[1] / 2

    def printJson(self, new_lines):
        print("""{\n"vertices" : """)
        print(f"""{np.array2string(
            np.array(self.vertices, dtype=float), precision=6, separator=", ", floatmode="fixed")},""")
        print(""""obstacles" : """)
        test = [str(i) for i in self.scene_objects]
        print(f"""[{", ".join([str(i) for i in self.scene_objects])}],""")
        # print(f"""{np.array2string(
        #     np.array(self.scene_objects, dtype=int), separator=", ")},""")
        print(""""boundaries" : """)
        print(f"""{np.array2string(
            np.array(self.scene_size, dtype=float), precision=6, separator=", ", floatmode="fixed")},""")
        print(""""topVertices" : """)
        print(f"""{np.array2string(
            np.array(self.top_vertices, dtype=int), separator=", ")},""")
        print(""""bottomVertices" : """)
        print(f"""{np.array2string(
            np.array(self.bottom_vertices, dtype=int), separator=", ")},""")
        print(""""lines" : """)
        print(f"""{np.array2string(np.array(new_lines, dtype=int), separator=", ")}\n}}""")

    def get_sympy_segment(self, x):
        return Segment(Point(x, -self.scene_size[1] / 2),
                       Point(x, self.scene_size[1] / 2))

    def intersections_for_v_id(self, v_id, obj):  # Int
        v = self.vertices[v_id]
        x, y = v[0], v[1]

        segment = Segment(Point2D(x, self.get_y_boundary()), Point2D(x, -self.get_y_boundary()))
        fo_array = [o for o in zip(self.objects_sympy, self.scene_objects_aabb,
                                   range(len(self.scene_objects))) if
                    o[1][0][0] <= x <= o[1][0][1]  # test auf x coord: o[1] = aabb, [][0][] = xvar
                    and o[2] != obj]
        if fo_array:
            return [i for i in
                    [[fo[2], Vcd2D.prune_is_list(intersection(fo[0], segment))] for fo in fo_array] if len(i[1]) > 0]
        else:
            return []

    @staticmethod
    def points_are_close(p1, p2):
        return euclidean(p1, p2) < 0.01

    @staticmethod
    def values_are_close(v1, v2):
        return -0.01 < v1 - v2 < 0.01

    def find_first_max_min(self, v_id, o_id):
        boundary_top = self.get_y_boundary()
        boundary_bottom = - boundary_top

        x = self.vertices[v_id][0]
        y = self.vertices[v_id][1]

        obj_up = [-1]
        obj_down = [-2]

        y_up = boundary_top
        y_down = boundary_bottom

        intersections = []

        if self.objects_sympy[o_id].encloses_point(Point2D(x, y + 0.01)):
            y_up = y
            obj_up = [o_id]
        if self.objects_sympy[o_id].encloses_point(Point2D(x, y - 0.01)):
            y_down = y
            obj_down = [o_id]
        if y_up == boundary_top or y_down == boundary_bottom:
            intersections = self.intersections_for_v_id(v_id, o_id)

        if len(intersections) != 0:
            y_values = []
            for i in intersections:
                y_values.extend([o[1]-y for o in i[1]])
            if y_up == boundary_top: ## no self intersection
                up_y_values = [i for i in y_values if i > 0.01]
                if up_y_values:
                    y_up = min(up_y_values) + y
                    obj_up = [i[0] for i in intersections if [k for k in i[1] if Vcd2D.values_are_close(y_up, k[1])]]
                    ##if not obj_up:
                    ##    print("Should not happen...")
                else:
                    obj_up = [-1]
            if y_down == boundary_bottom: ## i.e. not self intersecting in down direction
                down_y_values = [i for i in y_values if i < 0.01]
                if down_y_values:
                    y_down = max(down_y_values) + y
                    obj_down = [i[0] for i in intersections
                                if [k for k in i[1] if Vcd2D.values_are_close(y_down, k[1])]]
                    ##if not obj_down:
                    ##    print("Should not happen...")
                else:
                    obj_down = [-2]
        return [[obj_up[0], [x, y_up]], [obj_down[0], [x, y_down]]]

    @staticmethod
    def prune_is_list(l):
        new_list = []
        for i in l:
            if isinstance(i, Segment):
                new_list.extend([i.p1, i.p2])
            else:
                if isinstance(i, Point2D):
                    new_list.append(i)
                else:
                   print(f"unknown intersection: {i}")

        return new_list
