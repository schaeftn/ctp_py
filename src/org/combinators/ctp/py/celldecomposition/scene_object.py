import numpy as np
from abc import ABC
from sympy.geometry import Point, Polygon, convex_hull
import pymesh
import scipy.spatial.qhull as qhull

class SceneObject(ABC):
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



class SceneObject2D(SceneObject):
    def to_pymesh(self):
        return pymesh.form_mesh(self.vertices, self.triangles)

    def translate(self, vect):
        self.vertices = np.array([[x[0] + vect[0], x[1] + vect[1]] for x in self.vertices])

    #not tested yet
    def affine_transform(self, t):
        print(f"vertices: {self.vertices}")
        a = np.ones((np.size(self.vertices, axis=1)+1, 1))
        self.vertices = np.hsplit(np.array([np.matmul(t, x) for x in np.hstack((self.vertices, a))]), [2, 2])[0]

    def to_sympy(self):
        return Polygon(*self.vertices)

class SceneObject3D(SceneObject):
    def to_pymesh(self):
        return pymesh.form_mesh(self.vertices, self.triangles, self.voxels)

    def translate(self, vect):
        self.vertices = np.array([[x[0] + vect[0], x[1] + vect[1], x[2] + vect[2]] for x in self.vertices])

    def affine_transform(self, t):
        a = np.ones((np.size(self.vertices, 0), 1))
        self.vertices = np.hsplit(np.array([np.matmul(t, x) for x in np.hstack((self.vertices, a))]), [3, 1])[0]


stdV = np.array([[-0.5, -0.5, -0.5],
                 [0.5, -0.5, -0.5],
                 [0.5, 0.5, -0.5],
                 [-0.5, 0.5, -0.5],
                 [-0.5, -0.5, 0.5],
                 [0.5, -0.5, 0.5],
                 [0.5, 0.5, 0.5],
                 [-0.5, 0.5, 0.5]], dtype=float)


class SceneObjectBox3D(SceneObject3D):
    def __init__(self, v=stdV):
        super().__init__()
        self.vertices = v  # vertices supplied as arg or stdV with affine transformation
        self.triangles = np.array([[0, 2, 1],
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
                                   [6, 7, 4]])
        self.voxels = np.array([[0, 3, 7, 6],
                                [0, 3, 6, 2],
                                [0, 2, 6, 1],
                                [5, 0, 6, 1],
                                [5, 0, 4, 6],
                                [6, 0, 4, 7]])


class SceneObjectLine2d(SceneObject2D):
    def __init__(self, p1: list, p2: list):
        super().__init__()
        width = 0.005
        #print(f"p1: {p1}")
        self.vertices = np.array([[p1[0] - width, p1[1]],
                                  [p1[0] + width, p1[1]],
                                  [p2[0] + width, p2[1]],
                                  [p2[0] - width, p2[1]]])
        self.triangles = np.array([
            [0, 1, 3],
            [1, 2, 3],
        ])

        #self.vertices = np.array([[p1[0]-width, p1[1]],
        #                 [p1[0]+width, p1[1]],
        #                 [p2[0]+width, p2[1]],
        #                 [p2[0]-width, p2[1]]])
        #self.triangles = np.array([
        #    [0, 1, 3],
        #    [1, 2, 3],
        #])


class SceneObjectBox2d(SceneObject2D):
    def __init__(self):
        super().__init__()
        self.vertices = np.array([[-0.5, -0.5, 0.0],
                                 [0.5, -0.5, 0.0],
                                 [0.5, 0.5, 0.0],
                                 [-0.5, 0.5, 0.0]], dtype=float)  # vertex manipulation via affine transformation
        self.triangles = np.array([[0, 2, 1],
                                   [0, 3, 2]])


class SceneObjectPolygon(SceneObject2D):
    def __init__(self, p_vertices, p_triangles):
        super().__init__()
        self.vertices = p_vertices
        self.triangles = p_triangles


class SceneObjectCHull2D(SceneObject2D):
    def __init__(self, p_vertices):
        super().__init__()
        vertices = np.array(p_vertices)
        tri = pymesh.triangle()
        tri.points = vertices
        tri.split_boundary = False
        tri.verbosity = 0
        tri.run()
        self.vertices = tri.mesh.vertices
        self.triangles = tri.mesh.faces


class SceneObjectCHull(SceneObject3D):
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
    tri = pymesh.triangle()
    tri.points = get_test_vertices()
    tri.split_boundary = False
    tri.verbosity = 0
    tri.run()  # Execute triangle.
    mesh = tri.mesh  # output triangulation.
    return SceneObjectPolygon(p_vertices=mesh.vertices, p_triangles=mesh.faces)


def get_test_vertices():
    return np.array([(1, 1), (1, 2), (3, 2.5), (3, 0), (-1, 0)])


def get_test_polygon_sympy():
    return get_standard_polygon_so().to_sympy()


def get_sympy_point(v):
    return Point(v[0], v[1])
