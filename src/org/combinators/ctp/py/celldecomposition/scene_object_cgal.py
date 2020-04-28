import numpy as np
from abc import ABC
from sympy.geometry import Point, Polygon, convex_hull
import pymesh
import scipy.spatial.qhull as qhull

from CGAL.CGAL_Kernel import Point_2
from CGAL.CGAL_Mesh_2 import Mesh_2_Constrained_Delaunay_triangulation_2
from CGAL.CGAL_Mesh_2 import Delaunay_mesh_size_criteria_2
from CGAL import CGAL_Mesh_2
from CGAL.CGAL_Triangulation_2 import Triangulation_2
from CGAL.CGAL_Triangulation_2 import Triangulation_2_Vertex_circulator
from CGAL.CGAL_Triangulation_2 import Triangulation_2_Vertex_handle

class SceneObjectCGAL(ABC):
    @property
    def vertices(self):
        return self._vertices

    @property
    def triangles(self):
        return self._triangles

    @property
    def voxels(self):
        return self._voxels

    @triangles.setter
    def triangles(self, value):
        self._triangles = value

    @vertices.setter
    def vertices(self, value):
        self._vertices = value

    @voxels.setter
    def voxels(self, value):
        self._voxels = value

    # check: what kind of transform?
    def transform(self, t):
        self.vertices = np.array([np.cross(x, t) for x in self.vertices])
        # not used yet!


class SceneObject2dCGAL(SceneObjectCGAL):
    def to_mesh(self):
        cdt = Mesh_2_Constrained_Delaunay_triangulation_2()
        vertices = list(map(lambda ls: cdt.insert(Point_2(ls[0], ls[1])), self.vertices))
        for triangle in self.triangles:
            for i in range(len(triangle)):
                print(triangle[i])
                if (i + 1 == len(triangle)):
                    cdt.insert_constraint(vertices[triangle[i]], vertices[triangle[0]])
                else:
                    cdt.insert_constraint(vertices[triangle[i]], vertices[triangle[i + 1]])
        return cdt

    def translate(self, vect):
        self.vertices = [[x[0] + vect[0], x[1] + vect[1]] for x in self.vertices]
        # self.vertices = np.array([[x[0] + vect[0], x[1] + vect[1]] for x in self.vertices])

    # not tested yet
    #
    def affine_transform(self, t):
        print(f"vertices: {self.vertices}")
        vertices = np.array(self.vertices)
        a = np.ones((np.size(vertices, axis=1) + 1, 1))
        vertices = np.hsplit(np.array([np.matmul(t, x) for x in np.hstack((vertices, a))]), [2, 2])[0]
        self.vertices = vertices.tolist()

    def to_sympy(self):
        return Polygon(*self.vertices)


class SceneObject3dCGAL(SceneObjectCGAL):
    def to_pymesh(self):
        return pymesh.form_mesh(self.vertices, self.triangles, self.voxels)

    def translate(self, vect):
        self.vertices = [[x[0] + vect[0], x[1] + vect[1], x[2] + vect[2]] for x in self.vertices]
        # self.vertices = np.array([[x[0] + vect[0], x[1] + vect[1], x[2] + vect[2]] for x in self.vertices])

    def affine_transform(self, t):
        vertices = np.array(self.vertices)
        a = np.ones((np.size(vertices, 0), 1))
        vertices = np.hsplit(np.array([np.matmul(t, x) for x in np.hstack((vertices, a))]), [3, 1])[0]
        self.vertices = vertices.tolist()


stdV = np.array([[-0.5, -0.5, -0.5],
                 [0.5, -0.5, -0.5],
                 [0.5, 0.5, -0.5],
                 [-0.5, 0.5, -0.5],
                 [-0.5, -0.5, 0.5],
                 [0.5, -0.5, 0.5],
                 [0.5, 0.5, 0.5],
                 [-0.5, 0.5, 0.5]], dtype=float)


class SceneObjectBox3dCGAL(SceneObject3dCGAL):
    def __init__(self, v=stdV):
        super().__init__()
        self.vertices = v.tolist()  # vertices supplied as arg or stdV with affine transformation
        self.triangles = [[0, 2, 1],
                          [0, 3, 2],
                          [5, 0, 1],
                          [5, 4, 0],
                          [2, 6, 1],
                          [3, 6, 2],
                          [5, 1, 6],
                          [5, 6, 4],
                          [0, 7, 3],
                          [0, 4, 7],
                          [3, 7, 6],
                          [6, 7, 4]]
        self.voxels = np.array([[0, 3, 7, 6],
                                [0, 3, 6, 2],
                                [0, 2, 6, 1],
                                [5, 0, 6, 1],
                                [5, 0, 4, 6],
                                [6, 0, 4, 7]])


class SceneObjectLine2dCGAL(SceneObject2dCGAL):
    def __init__(self, p1: list, p2: list):
        super().__init__()
        width = 0.005
        # print(f"p1: {p1}")
        self.vertices = [[p1[0] - width, p1[1]],
                         [p1[0] + width, p1[1]],
                         [p2[0] + width, p2[1]],
                         [p2[0] - width, p2[1]]]
        self.triangles = [
            [0, 1, 3],
            [1, 2, 3],
        ]

        # self.vertices = np.array([[p1[0]-width, p1[1]],
        #                 [p1[0]+width, p1[1]],
        #                 [p2[0]+width, p2[1]],
        #                 [p2[0]-width, p2[1]]])
        # self.triangles = np.array([
        #    [0, 1, 3],
        #    [1, 2, 3],
        # ])


class SceneObjectBox2dCGAL(SceneObject2dCGAL):
    def __init__(self):
        super().__init__()
        self.vertices = [[-0.5, -0.5, 0.0],
                         [0.5, -0.5, 0.0],
                         [0.5, 0.5, 0.0],
                         [-0.5, 0.5, 0.0]]  # vertex manipulation via affine transformation
        self.triangles = [[0, 2, 1],
                          [0, 3, 2]]


class SceneObjectPolygonCGAL(SceneObject2dCGAL):
    def __init__(self, p_vertices, p_triangles):
        super().__init__()
        self.vertices = p_vertices
        self.triangles = p_triangles


class SceneObjectCHull2DCGAL(SceneObject2dCGAL):
    def __init__(self, p_vertices):
        super().__init__()

        vertices = list(map(lambda ls: Point_2(ls[0], ls[1]), p_vertices))

        cdt = Mesh_2_Constrained_Delaunay_triangulation_2()
        cdt.insert(vertices)

        CGAL_Mesh_2.refine_Delaunay_mesh_2(cdt, Delaunay_mesh_size_criteria_2())

        vertices = []
        for vh in cdt.finite_vertices():
            point = vh.point()
            vertex = []
            vertex.append(point.x())
            vertex.append(point.y())
            vertex.append(0.0)
            vertices.append(vertex)
        self.vertices = vertices

        faces = []
        for f in cdt.finite_faces():
            triangle = []
            triangle.append(f.vertex(0).point())
            triangle.append(f.vertex(1).point())
            triangle.append(f.vertex(2).point())
            faces.append(triangle)

        self.triangles = faces


class SceneObjectCHullCGAL(SceneObject3dCGAL):
    def __init__(self, p_vertices):
        super().__init__()
        c_hull = qhull.ConvexHull(p_vertices)
        print(f"c_hull.points: {c_hull.points}")
        print(f"c_hull.vertices: {c_hull.vertices}")
        print(f"c_hull.simplices: {c_hull.simplices}")
        mesh = pymesh.form_mesh(c_hull.points, c_hull.simplices)
        # pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/test_mesh_from_chull.obj", mesh)

        mesh_2 = pymesh.tetrahedralize(mesh, 0.5, radius_edge_ratio=2.0, facet_distance=1.0, feature_angle=120,
                                       with_timing=False)
        self.vertices = mesh_2.vertices
        self.triangles = mesh_2.faces
        self.voxels = mesh_2.voxels


def get_standard_polygon_so():
    vertices = list(map(lambda ls: Point_2(ls[0], ls[1]), get_test_vertices()))

    cdt = Mesh_2_Constrained_Delaunay_triangulation_2()
    cdt.insert(vertices)

    CGAL_Mesh_2.refine_Delaunay_mesh_2(cdt, Delaunay_mesh_size_criteria_2())

    vertices = []
    for vh in cdt.finite_vertices():
        point = vh.point()
        vertex = []
        vertex.append(point.x())
        vertex.append(point.y())
        vertex.append(0.0)
        vertices.append(vertex)

    faces = []
    for f in cdt.finite_faces():
        triangle = []
        triangle.append(f.vertex(0).point())
        triangle.append(f.vertex(1).point())
        triangle.append(f.vertex(2).point())
        faces.append(triangle)

    return SceneObjectPolygonCGAL(p_vertices=vertices, p_triangles=faces)


def get_test_vertices():
    return [[1, 1], [1, 2], [3, 2.5], [3, 0], [-1, 0]]


def get_test_polygon_sympy():
    return get_standard_polygon_so().to_sympy()


def get_sympy_point(v):
    return Point(v[0], v[1])
