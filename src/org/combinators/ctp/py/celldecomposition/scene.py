import sys

from abc import abstractmethod
from abc import ABC
import numpy as np
import pymesh

class Scene(ABC):
    def __init__(self):
        self._dimensionality = None
        self._scene_objects = None
        self._scene_size = None

    @property
    def scene_objects(self):
        return self._scene_objects

    @property
    def scene_size(self):
        return self._scene_size

    @property
    def dimensionality(self):
        return self._dimensionality

    @dimensionality.setter
    def dimensionality(self, value):
        self._dimensionality = value

    @scene_objects.setter
    def scene_objects(self, value):
        self._scene_objects = value

    @scene_size.setter
    def scene_size(self, value):
        self._scene_size = value

    # def get_objects_test(self):
    #     for i in self.geometry.values():
    #         print(f"isBox: {isinstance(i, fcl.Box)}")
    #         print(f"isBVHModel: {isinstance(i, fcl.BVHModel)}")

    def export_model(self):
        scene_mesh = self.transform_to_pymesh()
        pymesh.save_mesh("/home/tristan/projects/ctp_py/resources/mesh_processing/scene_mesh_out.obj", scene_mesh)

    @abstractmethod
    def get_scene_size_pym(self):
        ...

    def transform_to_pymesh(self):
        output_mesh = self.get_scene_size_pym()
        for o in self.scene_objects:
            output_mesh = pymesh.boolean(output_mesh, o.to_pymesh(), operation="difference")
        return output_mesh


class Scene3D(Scene):
    def get_scene_size_pym(self):
        ar = self.scene_size
        return pymesh.generate_box_mesh(np.array([-ar[0]/2, -ar[1]/2, -ar[2]/2]),
                                        np.array([ar[0]/2, ar[1]/2, ar[2]/2]))

    def transform_to_pymesh(self):
        output_mesh = self.get_scene_size_pym()
        for o in self.scene_objects:
            output_mesh = pymesh.boolean(output_mesh, o.to_pymesh(), operation="difference")
        return output_mesh

    # def __init__(self):
    #     self.dimensionality = 3
    #     box0 = SceneObjectBox3D(0.2, 0.2, 0.1)
    #     box0.affine_transform(
    #         np.array([[0.2, 0.0, 0.0, -0.326], [0.0, 0.2, 0.0, 0.17], [0.0, 0.0, 0.1, -0.056], [0.0, 0.0, 0.0, 1.0]]))
    #     box1 = SceneObjectBox3D(0.1, 0.3, 0.3)
    #     box1.affine_transform(np.array(
    #         [[0.053112146, 0.07076661, 0.2441393, 0.045], [0.043301277, 0.22499998, -0.15000004, -0.17],
    #          [-0.072829254, 0.18538368, 0.088859476, -0.05], [0.0, 0.0, 0.0, 1.0]]))
    #     box2 = SceneObjectBox3D(0.1, 0.1, 0.1)
    #     box2.affine_transform(np.array(
    #         [[0.07071068, -0.07071068, 0.0, 0.307], [0.07071068, 0.07071068, 0.0, -0.222], [0.0, 0.0, 0.1, 0.168],
    #          [0.0, 0.0, 0.0, 1.0]]))
    #     self.scene_objects = [box0, box1, box2]

    def __init__(self, scene_objects, scene_size=np.array([10, 10, 10])):
        Scene.__init__(self)
        self.dimensionality = 3
        self.scene_objects = scene_objects
        self.scene_size = scene_size
        self.scene_bounds = np.array(
            [[-scene_size[0] / 2, scene_size[0] / 2], [-scene_size[1] / 2, scene_size[1] / 2],
             [-scene_size[2] / 2, scene_size[2] / 2]])


class Scene2D(Scene):
    def __init__(self, scene_objects, scene_size=np.array([10, 10])):
        Scene.__init__(self)
        self.dimensionality = 2
        self.scene_objects = scene_objects
        self.scene_bounds = np.array(
            [[-scene_size[0] / 2, scene_size[0] / 2], [-scene_size[1] / 2, scene_size[1] / 2]])
        self.scene_size = scene_size

    def get_scene_size_pym(self):
        ar = self.scene_size
        x_min, x_max = -ar[0]/2, ar[0]/2
        y_min, y_max = -ar[1]/2, ar[1]/2
        faces = [[0, 1, 3],
                 [1, 2, 3]]
        return pymesh.form_mesh(np.array(
            [[x_min, y_min],
             [x_max, y_min],
             [x_max, y_max],
             [x_min, y_max]]),
            np.array(faces))

    def get_top_y_boundary(self):
        return (lambda x: x[1]/2)(self.scene_size)

    def get_bottom_y_boundary(self):
        return (lambda x: -x[1]/2)(self.scene_size)

    @property
    def objects_sympy(self):
        if hasattr(self, "_objects_sympy"):
            return self._objects_sympy

        self._objects_sympy = [x.to_sympy() for x in self.scene_objects]
        return self._objects_sympy

    @objects_sympy.setter
    def objects_sympy(self, value):
        self._objects_sympy = value
