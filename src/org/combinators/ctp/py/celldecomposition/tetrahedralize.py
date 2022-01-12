import sys
import numpy as np
import pyvista

sys.path.append("/home/tristan/projects/cell_decomposition_py/src")

from org.combinators.ctp.py.celldecomposition.scene import *

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

def tetrahedralize(scene_objects, scene_size):
    scene = Scene3D(scene_objects, scene_size)
    ar = scene_size
    tup = (-ar[0] / 2, ar[0] / 2, -ar[1] / 2, ar[1] / 2, -ar[2] / 2, ar[2] / 2)

    scene_mesh = pyvista.Box(bounds=tup, quads=False)

    # pymesh wegspeichern, mit pyvista laden, dann tetrahedra check
    print(f"box: {scene_mesh.is_all_triangles}")
    for o in scene_objects:
        pvm = pyvista.PolyData(var_inp=o.vertices, faces=o.triangles).triangulate(inplace=False)
        print(f"pvm: {pvm.is_all_triangles}")
        scene_mesh.boolean_difference(pvm)

    grid = scene_mesh.delaunay_3d()
    print(f"grid: {grid}")
    tetgen = pymesh.tetgen()
    tetgen.points = scene_mesh.vertices
    tetgen.triangles = scene_mesh.faces
    tetgen.verbosity = 0
    tetgen.run()
    mesh_2 = tetgen.mesh

    #pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/tetfinalTest2.obj", output_mesh)
    #pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/tetfinal.obj", mesh_2)
    #pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/scene_mesh.obj", scene_mesh)
    #
    # mesh_2 = pymesh.tetrahedralize(scene_mesh, 10.0,
    #                                radius_edge_ratio=4.0, facet_distance=4.0, feature_angle=120, engine='auto',
    #                                with_timing=False)

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
