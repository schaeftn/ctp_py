import sys
import numpy as np

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene import Scene3D, Scene2D
from org.combinators.ctp.py.celldecomposition.scene_object import *

np.set_printoptions(threshold=sys.maxsize)


def triangulate2d_params(scene_objects, scene_size):
    scene = Scene2D(scene_objects, scene_size)
    scene_mesh = scene.transform_to_pymesh()

    triangle = pymesh.triangle()
    triangle.points = scene_mesh.vertices
    triangle.triangles = scene_mesh.faces
    triangle.max_area = 1000
    triangle.split_boundary = True
    triangle.auto_hole_detection = False
    triangle.verbosity = 0
    triangle.run()
    mesh_2 = triangle.mesh

    print("""{ "vertices" : """)
    print(f"""{np.array2string(mesh_2.vertices, precision=6, separator=", ", floatmode="fixed")},""")
    print(""" "cells" : """)
    print(f"""{np.array2string(mesh_2.faces, separator=", ")}}}""")
