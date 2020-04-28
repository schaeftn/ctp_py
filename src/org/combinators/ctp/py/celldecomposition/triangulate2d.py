import sys
import numpy as np

sys.path.append("/home/tristan/projects:/home/tristan/projects/cell_decomposition_py/src/")

from org.combinators.ctp.py.celldecomposition.scene import Scene3D, Scene2D
from org.combinators.ctp.py.celldecomposition.scene_object import *

np.set_printoptions(threshold=sys.maxsize)


def triangulate2d(scene_objects, scene_size):
    import time

    # start = time.time()
    scene = Scene2D(scene_objects, scene_size)
    scene_mesh = scene.transform_to_pymesh()
    # end = time.time()
    # print(end - start)

    print("""{ "vertices" : """)
    print(f"""{np.array2string(scene_mesh.vertices, precision=6, separator=", ", floatmode="fixed")},""")
    print(""" "cells" : """)
    print(f"""{np.array2string(scene_mesh.faces, separator=", ")}}}""")
