import sys

from abc import abstractmethod
from abc import ABC
import numpy as np
import pymesh



#Create input polyhedron

basefolder = "/home/tristan/projects/ctp_py/"
polyhedron=pymesh.load_mesh(basefolder+"resources/Pyramid.stl")
polyhedron2=pymesh.load_mesh(basefolder+"resources/Sphere.stl")
lines = polyhedron.vertices
newmesh = pymesh.minkowski_sum(polyhedron2, polyhedron.vertices)
pymesh.meshio.save_mesh(basefolder + "resources/minkowski_out.stl" , newmesh)
chull_minkowski = pymesh.convex_hull(newmesh, engine='auto', with_timing=False)
pymesh.meshio.save_mesh(basefolder + "resources/chull_minkowski.stl" , chull_minkowski)
