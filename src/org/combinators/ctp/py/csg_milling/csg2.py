import pymesh
import numpy as np
import sys

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

outer_box = pymesh.generate_box_mesh(np.array([0.0, 0.0, 0.0]), np.array([75.0, 50.0, 3.0]))

inner_area_edge_radius = 6.0
min_inner_area_edge_radius = 4.0

cy1 = pymesh.generate_cylinder(np.array([11.0, 11.0, 0.0]), np.array([11.0, 11.0, 3.0]), inner_area_edge_radius,inner_area_edge_radius, num_segments=36)
cy2 = pymesh.generate_cylinder(np.array([11.0, 39.0, 0.0]), np.array([11.0, 39.0, 3.0]), inner_area_edge_radius,inner_area_edge_radius, num_segments=36)
cy3 = pymesh.generate_cylinder(np.array([39.0, 39.0, 0.0]), np.array([39.0, 39.0, 3.0]), inner_area_edge_radius,inner_area_edge_radius, num_segments=36)
cy4 = pymesh.generate_cylinder(np.array([39.0, 11.0, 0.0]), np.array([39.0, 11.0, 3.0]),inner_area_edge_radius,inner_area_edge_radius, num_segments=36)

tree = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": cy1}),
        pymesh.CSGTree({"mesh": cy2}),
        pymesh.CSGTree({"mesh": cy3}),
        pymesh.CSGTree({"mesh": cy4})]
   })

inner_area = pymesh.convex_hull(tree.mesh, engine='auto', with_timing=False)

bot_mesh = pymesh.load_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/CSGCMP2.stl")

basic_mesh = pymesh.CSGTree({"difference":
       [pymesh.CSGTree({"mesh": outer_box}),
        pymesh.CSGTree({"mesh": inner_area})]
   })

basic_mesh_2 = pymesh.CSGTree({"union":
       [basic_mesh,
        pymesh.CSGTree({"mesh": bot_mesh})]
   })

pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/02_basic_mesh.stl", basic_mesh.mesh)
pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/02_basic_mesh_2.stl", basic_mesh_2.mesh)

right_box = pymesh.generate_box_mesh(np.array([45.0, 30.0, 0.0]), np.array([65.0, 40.0, 3.0]))

right_upper_z1 = pymesh.generate_cylinder(np.array([55.0, 41.0, 0.0]), np.array([55.0, 41.0, 3.0]), min_inner_area_edge_radius,min_inner_area_edge_radius, num_segments=36)
right_upper_z2 = pymesh.generate_cylinder(np.array([66.0, 41.0, 0.0]), np.array([66.0, 41.0, 3.0]), min_inner_area_edge_radius,min_inner_area_edge_radius, num_segments=36)
right_upper_z3 = pymesh.generate_cylinder(np.array([60.0, 35.0, 0.0]), np.array([60.0, 35.0, 3.0]), min_inner_area_edge_radius,min_inner_area_edge_radius, num_segments=36)


right_upper = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": right_upper_z1}),
        pymesh.CSGTree({"mesh": right_upper_z2}),
        pymesh.CSGTree({"mesh": right_upper_z3})]
   })

right_upper_hull = pymesh.convex_hull(right_upper.mesh, engine='auto', with_timing=False)

right_lower_box = pymesh.generate_box_mesh(np.array([55.0, 15.0, 0.0]), np.array([65.0, 30.0, 3.0]))

right_lower_z1 = pymesh.generate_cylinder(np.array([55.0, 9.0, 0.0]), np.array([55.0, 9.0, 3.0]), min_inner_area_edge_radius,min_inner_area_edge_radius, num_segments=36)
right_lower_z2 = pymesh.generate_cylinder(np.array([66.0, 9.0, 0.0]), np.array([66.0, 9.0, 3.0]), min_inner_area_edge_radius,min_inner_area_edge_radius, num_segments=36)
right_lower_b3 = pymesh.generate_box_mesh(np.array([55.0, 10.0, 0.0]), np.array([65.0, 15.0, 3.0]))

right_lower = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": right_lower_z1}),
        pymesh.CSGTree({"mesh": right_lower_z2}),
        pymesh.CSGTree({"mesh": right_lower_b3})]
   })

right_lower_hull = pymesh.convex_hull(right_lower.mesh, engine='auto', with_timing=False)

right_lower_csg = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": right_lower_box}),
        pymesh.CSGTree({"mesh": right_lower_hull})]
   })


right_csg = pymesh.CSGTree({"union":
       [
        pymesh.CSGTree({"mesh": right_box}),
        pymesh.CSGTree({"mesh": right_upper_hull}),
        right_lower_csg]
   })

isle_box = pymesh.generate_box_mesh(np.array([10.0, 22.0, 0.0]), np.array([25.0, 37.0, 3.0]))
isle_z_upper_right = pymesh.generate_cylinder(np.array([25.0, 29.0, 0.0]), np.array([25.0, 29.0, 3.0]), 7.0,7.0, num_segments=36)
isle_z_lower_right = pymesh.generate_cylinder(np.array([32.0, 23.0, 0.0]), np.array([32.0, 23.0, 3.0]), 1.0, 1.0,
                                              num_segments=12)
isle_csg = pymesh.CSGTree({"union":
       [
        pymesh.CSGTree({"mesh": isle_box}),
        pymesh.CSGTree({"mesh": isle_z_upper_right}),
        pymesh.CSGTree({"mesh": isle_z_lower_right})]
   })

isle_hull = pymesh.convex_hull(isle_csg.mesh, engine='auto', with_timing=False)



isle_inner_upper_left = pymesh.generate_cylinder(np.array([17.0, 30.0, 0.0]), np.array([17.0, 30.0, 3.0]),
                                                 5.0, 5.0, num_segments=24)

isle_inner_upper_right = pymesh.generate_cylinder(np.array([25.0, 29.0, 0.0]), np.array([25.0, 29.0, 3.0]),
                                                  5.0,5.0, num_segments=24)

isle_inner_upper_middle = pymesh.generate_cylinder(np.array([23.0, 25.0, 0.0]), np.array([23.0, 25.0, 3.0]),
                                                 4.0, 4.0, num_segments=24)

isle_inner_csg = pymesh.CSGTree({"union":
       [pymesh.CSGTree({"mesh": isle_inner_upper_left}),
        pymesh.CSGTree({"mesh": isle_inner_upper_right}),
        pymesh.CSGTree({"mesh": isle_inner_upper_middle})
        ]})

isle_inner_ch = pymesh.convex_hull(isle_inner_csg.mesh, engine='auto', with_timing=False)

isle_csg = pymesh.CSGTree({"difference":
       [pymesh.CSGTree({"mesh": isle_hull}),
        pymesh.CSGTree({"mesh": isle_inner_ch})]
   })

basic_mesh_2a = pymesh.CSGTree({"union":
       [basic_mesh_2,
        isle_csg]
   })

basic_mesh_3 = pymesh.CSGTree({"difference":
       [basic_mesh_2a,
        pymesh.CSGTree({"mesh": right_csg})]
   })

pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/02_basic_mesh_3.stl", basic_mesh_3.mesh)

print("""{ "vertices" : """)
print(f"""{np.array2string(basic_mesh_2.mesh.vertices, precision=6, separator=", ", floatmode="fixed")},""")
print(""" "obstacles" : """)
print(f"""{np.array2string(basic_mesh_2.mesh.faces, separator=", ")},""")
print(""" "boundaries" : """)
print(f"""[]}}""")

