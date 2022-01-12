import pymesh
import numpy as np
import sys


treeFinal = pymesh.load_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/01_treeFinal.stl")

print("""{ "vertices" : """)
print(f"""{np.array2string(treeFinal.mesh.vertices, precision=6, separator=", ", floatmode="fixed")},""")
print(""" "obstacles" : """)
print(f"""{np.array2string(treeFinal.mesh.faces, separator=", ")},""")
print(""" "boundaries" : """)
print(f"""[]}}""")

