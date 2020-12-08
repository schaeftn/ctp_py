import pymesh
import numpy as np
import sys

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)


cy = pymesh.generate_cylinder(np.array([5.0, 5.0, 0.0]), np.array([5.0, 5.0, 3.0]), 5.0, 5.0, num_segments=36)
cy2 = pymesh.generate_cylinder(np.array([95.0, 95.0, 0.0]), np.array([95.0, 95.0, 3.0]), 5.0, 5.0, num_segments=36)
cy3 = pymesh.generate_cylinder(np.array([95.0, 5.0, 0.0]), np.array([95.0, 5.0, 3.0]), 5.0, 5.0, num_segments=36)
cy4 = pymesh.generate_cylinder(np.array([5.0, 95.0, 0.0]), np.array([5.0, 95.0, 3.0]), 5.0, 5.0, num_segments=36)

tree = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": cy}),
        pymesh.CSGTree({"mesh": cy2}),
        pymesh.CSGTree({"mesh": cy3}),
        pymesh.CSGTree({"mesh": cy4})]
   })

ch = pymesh.convex_hull(tree.mesh, engine='auto', with_timing=False)


pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_cy.stl", cy)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_tree.stl", tree.mesh)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_ch.stl", ch)

circ1 = pymesh.generate_cylinder(np.array([60.0, 40, 0.0]),
                                 np.array([60.0, 40.0, 3.0]), 10.0, 10.0, num_segments=36)
b1 = pymesh.generate_box_mesh(np.array([50.0, 15, 0.0]), np.array([70.0, 40.0, 3.0]))
b2 = pymesh.generate_box_mesh(np.array([60.0, 20.0, 0.0]), np.array([85.0, 50.0, 3.0]))

tree2 = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": b1}),
        pymesh.CSGTree({"mesh": b2}),
        pymesh.CSGTree({"mesh": circ1})]
   })

treeFinal = pymesh.CSGTree({"difference": [pymesh.CSGTree({"mesh": ch}), tree2]})

pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_b2.stl", b2)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_b1.stl", b1)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_circ1.stl", circ1)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_t2.stl", tree2.mesh)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/00_treeFinal.stl", treeFinal.mesh)

print("""{ "vertices" : """)
print(f"""{np.array2string(treeFinal.mesh.vertices, precision=6, separator=", ", floatmode="fixed")},""")
print(""" "obstacles" : """)
print(f"""{np.array2string(treeFinal.mesh.faces, separator=", ")},""")
print(""" "boundaries" : """)
print(f"""[]}}""")

