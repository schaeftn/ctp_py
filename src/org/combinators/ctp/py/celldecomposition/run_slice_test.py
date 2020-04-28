import sys

from scipy.spatial.distance import euclidean

sys.path.append("/home/tristan/projects/ctp_py/src")

import math
import numpy as np

from org.combinators.ctp.py.celldecomposition.tetrahedralize import *
from org.combinators.ctp.py.celldecomposition.scene_object import *
from scipy.spatial import ConvexHull


np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)


def valid_function(x, y, z, alpha, beta):
    if 0.1 < x < 0.5 and 0.4 < y < 0.5:
        if alpha < 10 and beta < 10:
            return False
    return True


def get_resolutions(p_0, p_1, p_2, min_distance):
    def get_resolution(point_1):
        return math.ceil(euclidean(np.array(p_0), np.array(point_1)) / min_distance)

    return get_resolution(p_1), get_resolution(p_2)

def test_route(points, min_distance):
    np_4 = np.delete(points, 1, 1)  # delete first element in dimension 1
    y_val = points[0][1]
    ch = ConvexHull(np.array(np_4))
    print(f"simplices: {ch.simplices}")
    print(f"number of Vertices: {ch.vertices}")

    asd = [[element for element in simplex if element != ch.vertices[0]] for simplex in ch.simplices if
           ch.vertices[0] in simplex]
    r_array = np.array(asd).flatten()

    print(f"asd: {r_array}")
    point_array = get_points(ch.points[ch.vertices[0]], ch.points[r_array[0]], ch.points[r_array[1]], min_distance)
    point_array[1::2] = [np.flip(x, axis=0) for x in point_array[1::2]]
    print(f"point_array {point_array}")
    path = point_array.reshape(-1, 2)
    print(f"Path: {path}")
    r_array_final = [[p[0], y_val, p[1]] for p in path]
    print(r_array_final)
    return(r_array_final)
    # calc N from min_distance, dreieck
    #add y element,


def get_point(p_0, p_1, p_2, u, v):
    return p_0 + (u * (p_1 - p_0)) + (v * (p_2 - p_0))


def get_points_obs(p_0, p_1, p_2, n):
    (resolution_1, resolution_2) = get_resolutions(p_0, p_1, p_2, n)
    print(f"resolutions {resolution_1}, {resolution_2}")
    return np.array(
        [[get_point(p_0, p_1, p_2, (u + 0.5) / resolution_1, v)
          for v in [0.5 / resolution_2, 1 - 0.5 / resolution_2]] for u in range(resolution_1)])

def get_points(p_0, p_1, p_2, n):
    (resolution_1, resolution_2) = get_resolutions(p_0, p_1, p_2, n) ## Check if r1<r2?
    print(f"resulutions {resolution_1}, {resolution_2}")
    def get_v_max(u):
        return 1 - u
    return np.array(
        [[get_point(p_0, p_1, p_2, (u + 0.5) / resolution_1, v)
          for v in [min(get_v_max((u + 0.5)/resolution_1) - 0.5 / resolution_2, 0.5 / resolution_2),
                    get_v_max((u + 0.5)/resolution_1) - 0.5 / resolution_2]] for u in range(resolution_1)])
## TODO Sonderfall: Px, letzter Punkt v


def test_route_mult(m, dist):
    path = []
    for i in m:
        path.append(test_route(i.vertices, dist))

    path[1::2] = [np.flip(x, axis=0) for x in path[1::2]]
    p = np.array(path)

    return p.reshape(-1, 3)


def run_slice_test():
    b0 = SceneObjectBox3D(np.array([[-52.5, -52.5, -55.0], [-27.5, -52.5, -55.0], [-27.5, -27.5, -55.0], [-52.5, -27.5, -55.0], [-52.5, -52.5, 55.0], [-27.5, -52.5, 55.0], [-27.5, -27.5, 55.0], [-52.5, -27.5, 55.0]]))
    mesh2 = pymesh.slice_mesh(b0.to_pymesh(), np.array([0.0, 1.0, 0.0, ]), 3)

    print(f"mesh slide vertices: {mesh2[0].vertices}")
    print("now:::")
    print(test_route_mult(mesh2, 0.5))
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/boxMesh.msh", b0.to_pymesh())
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/box0.msh", mesh2[0])
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/box1.msh", mesh2[1])
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/box2.msh", mesh2[2])

    b1 = SceneObjectBox3D(np.array([[-42.600002, 7.0, -10.6], [-22.600002, 7.0, -10.6], [-22.600002, 27.0, -10.6], [-42.600002, 27.0, -10.6], [-42.600002, 7.0, -0.6000004], [-22.600002, 7.0, -0.6000004], [-22.600002, 27.0, -0.6000004], [-42.600002, 27.0, -0.6000004]]))
    b2 = SceneObjectBox3D(np.array([[30.7, -29.271069, 11.799999], [37.77107, -22.2, 11.799999], [30.7, -15.128933, 11.799999], [23.628933, -22.2, 11.799999], [30.7, -29.271069, 21.8], [37.77107, -22.2, 21.8], [30.7, -15.128933, 21.8], [23.628933, -22.2, 21.8]]))
    b3 = SceneObjectBox3D(np.array([[-13.900902, -22.915062, -15.070695], [-8.589687, -18.584934, -22.35362], [-1.5130267, 3.9150658, -3.8152523], [-6.8242416, -0.41506195, 3.4676723], [10.513027, -37.915066, -6.1847477], [15.824242, -33.584938, -13.467672], [22.900902, -11.084939, 5.070695], [17.589687, -15.415067, 12.353621]]))
    scene_objects = [b0, b1, b2, b3]
    scene_size = [100.0, 100.0, 100.0]
    #tetrahedralize(scene_objects, scene_size)


if __name__ == '__main__':
    run_slice_test()