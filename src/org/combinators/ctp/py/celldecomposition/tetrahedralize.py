import sys
import numpy as np

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene import *

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

def tetrahedralize(scene_objects, scene_size):
    scene = Scene3D(scene_objects, scene_size)
    scene_mesh = scene.transform_to_pymesh()

    tetgen = pymesh.tetgen()
    tetgen.points = scene_mesh.vertices
    tetgen.triangles = scene_mesh.faces
    tetgen.tetrahedra = scene_mesh.voxels
    tetgen.max_tet_volume = 2000000.0
    tetgen.max_radius_edge_ratio = 2000000.0
    tetgen.verbosity = 0
    tetgen.run()
    mesh_2 = tetgen.mesh

    # mesh_2 = pymesh.tetrahedralize(scene_mesh, 10.0, radius_edge_ratio=4.0, facet_distance=4.0, feature_angle=120, engine='auto',
    #                       with_timing=False)

    # pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/meshTet.msh", mesh_2)
    # pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/scene_mesh.msh", scene_mesh)

    # print("""{\n"vertices" : """)
    # print(f"""{np.array2string(mesh_2.vertices, precision=6, separator=", ", floatmode="fixed")}""")
    # print(""",\n"faces" : """)
    # print(f"""{np.array2string(mesh_2.faces, separator=", ")}""")
    # print(""",\n"voxels" : """)
    # print(f"""{np.array2string(mesh_2.voxels, separator=", ")}\n}}""")

    print("""{ "vertices" : """)
    print(f"""{np.array2string(mesh_2.vertices, precision=6, separator=", ", floatmode="fixed")},""")
    print(""" "cells" : """)
    print(f"""{np.array2string(mesh_2.voxels, separator=", ")}}}""")

    return mesh_2
