from combinators.ctp.py.celldecomposition.scene import *
from pymesh import Mesh
from scipy.spatial import distance
import sys

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene_object import *


def mesh_to_graph_3d(m: Mesh):
    center_points = np.array([k.mean(axis=0) for k in (m.vertices[i] for i in m.voxels)])  # TODO too many
    weighted_edges = find_neighbor_weights(m, center_points)
    print(f"weighted_edges: {weighted_edges}")
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/mesh_to_graph_test.msh", m)


def mesh_to_graph_2d(m: Mesh):
    center_points = np.array([k.mean(axis=0) for k in (m.vertices[i] for i in m.faces)])  # TODO test
    weighted_edges = find_neighbor_weights(m, center_points)
    print(f"weighted_edges: {weighted_edges}")
    pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/mesh_to_graph_test.msh", m)


def voxel_distance(v1: np.array, v2: np.array):
    return distance.euclidean(v1, v2)


def cells_are_neighbours(v1, v2):
    print(f"vertices: v1: {v1}, v2: {v2}")
    print(f"vertices_are_neighbours: {v1} ---  {v2}: {np.intersect1d(v1, v2).shape[0] == 3}")
    return np.intersect1d(v1, v2).shape[0] == 3


def find_neighbor_weights(m: Mesh, center_points):
    assert (len(m.vertices) == len(np.unique(m.vertices, axis=1)))
    assert len(m.voxels) == len(center_points)
    neighbours = ((x, y) for x in range(len(m.voxels)) for y in range(len(m.voxels)) if
                  cells_are_neighbours(m.voxels[x], m.voxels[y]))
    gen_expression_2 = [(v1, v2, voxel_distance(center_points[v1], center_points[v2])) for v1, v2 in neighbours]
    return np.array(gen_expression_2)