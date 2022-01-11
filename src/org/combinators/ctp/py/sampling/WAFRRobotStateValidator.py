import numpy as np
import stl
from stl import mesh
import trimesh
import fcl
from pytransform3d import urdf
from pytransform3d.transformations import transform
from timeit import default_timer as timer
from trimesh.collision import CollisionManager

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys



class WAFRRobotStateValidator(ob.StateValidityChecker):
    def __init__(self,package,environment):
        self.tm = urdf.UrdfTransformManager()
        self.meshes = []
        self.clms = []
        with open(package + "/urdf/URDF.urdf", "r") as f:
            self.tm.load_urdf(f.read())       
        for i,meshname in enumerate(list(filter(lambda x: x[0].isdigit(), self.tm.nodes))):
            tmesh = trimesh.load_mesh((package + "/meshes/{}.stl".format(meshname)))
            clm = CollisionManager()
            clm.add_object(i,tmesh)
            self.clms.append(clm)
            self.meshes.append(tmesh)  
        self.env_clm = CollisionManager()
        self.env_clm.add_object('env',trimesh.load_mesh(environment))
        self.tfts = list(filter(lambda x: x[0].isdigit(), self.tm.nodes))
        
    def getSamplingSpace(self):
        space = ob.RealVectorStateSpace()
        for joint in self.tm._joints:
           space.addDimension(self.tm.get_joint_limits(joint)[0],self.tm.get_joint_limits(joint)[1])
        return space
        
    def setJointConfiguration(self,jointConfig):
        if len(jointConfig) <= len(self.tm._joints):
            for i,j in enumerate(jointConfig):
                self.tm.set_joint("joint_{}".format(i), j * np.pi)
            self.tfs = []
            for i,frame in enumerate(self.tfts):
                tf = self.tm.get_transform(frame,"0_link")
                self.tfs.append(tf)
            for tf in self.tfs:
                tf[0:3,3] *= 1000          
        else:
            print("Dim of jointConfig is higher than Dim of robot")
           
        
    def isValid(self, *args, **kwargs):
        ompl_state = args[0]
        self.setJointConfiguration(ompl_state.values)
        
        for i,m in enumerate(self.meshes): 
            self.clms[i].set_transform(i,self.tfs[i])
        collision = False
        #This is 7 times faster than whatever the wrapper implemented like this
        for i in range(len(self.meshes)-2):
            for k in range(i+2,len(self.meshes)):
                if self.clms[i].in_collision_other(self.clms[k]):
                    collision = True
        collision_objects = []
        if(collision):
            return False
        if(not collision):       
            for id,clm in enumerate(self.clms):
                req = fcl.CollisionRequest()
                result = fcl.CollisionResult()
                ret = fcl.collide(clm._objs[id]['obj'], self.env_clm._objs['env']['obj'], req, result)             
                if result.is_collision:
                    return False
        return True       