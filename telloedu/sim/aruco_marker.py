import os
import tempfile
import cv2
import numpy as np
import pybullet as p


class ArucoMarker:
    """Textured ArUco target placed in the PyBullet world.

    The marker is a square mesh in the local Y-Z plane. At yaw=180 deg it
    faces a drone near the origin when the marker is placed on the +X axis.
    """

    def __init__(self, marker_id=23, size=0.30, position=(2.0, 0.0, 1.0),
                 yaw=180.0, dictionary=cv2.aruco.DICT_4X4_50, client_id=0):
        self.marker_id = int(marker_id)
        self.size = float(size)
        self.client_id = int(client_id)
        self.dictionary_id = dictionary
        self.texture_file = self._generate_marker_texture()
        self.mesh_file = self._generate_square_mesh()
        self.body_id = self._create_marker(position, yaw)

    def _generate_marker_texture(self):
        dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        marker = cv2.aruco.generateImageMarker(dictionary, self.marker_id, 512)
        # Keep the physical marker definition equal to `size`: the white quiet
        # zone is intentionally small and part of the rendered board only.
        marker = cv2.copyMakeBorder(marker, 32, 32, 32, 32, cv2.BORDER_CONSTANT, value=255)
        filename = os.path.join(tempfile.gettempdir(), f"telloedu_aruco_{self.marker_id}.png")
        cv2.imwrite(filename, marker)
        return filename

    def _generate_square_mesh(self):
        # Unit square in Y-Z plane with explicit UV coordinates. Scaling by
        # `size` in createVisualShape gives the requested physical side length.
        filename = os.path.join(tempfile.gettempdir(), f"telloedu_aruco_plane_{self.marker_id}.obj")
        obj = """v 0 -0.5 -0.5\nv 0 0.5 -0.5\nv 0 0.5 0.5\nv 0 -0.5 0.5\nvt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\nf 3/3 2/2 1/1\nf 4/4 3/3 1/1\n"""
        with open(filename, "w", encoding="utf-8") as f:
            f.write(obj)
        return filename

    def _create_marker(self, position, yaw):
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=self.mesh_file,
            meshScale=[self.size * (576.0 / 512.0)] * 3,
            rgbaColor=[1, 1, 1, 1],
            physicsClientId=self.client_id,
        )
        orientation = p.getQuaternionFromEuler([0.0, 0.0, np.deg2rad(yaw)])
        body_id = p.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            baseOrientation=orientation,
            physicsClientId=self.client_id,
        )
        texture_id = p.loadTexture(self.texture_file, physicsClientId=self.client_id)
        p.changeVisualShape(body_id, -1, textureUniqueId=texture_id, physicsClientId=self.client_id)
        self.texture_id = texture_id
        return body_id

    def set_position(self, x, y, z):
        _, orientation = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(self.body_id, [x, y, z], orientation,
                                          physicsClientId=self.client_id)

    def set_yaw(self, yaw):
        position, _ = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client_id)
        orientation = p.getQuaternionFromEuler([0.0, 0.0, np.deg2rad(yaw)])
        p.resetBasePositionAndOrientation(self.body_id, position, orientation,
                                          physicsClientId=self.client_id)

    def set_pose(self, position, yaw=180.0):
        orientation = p.getQuaternionFromEuler([0.0, 0.0, np.deg2rad(yaw)])
        p.resetBasePositionAndOrientation(self.body_id, position, orientation,
                                          physicsClientId=self.client_id)

    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client_id)
        return np.asarray(position)

    def get_pose(self):
        position, orientation = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client_id)
        euler = p.getEulerFromQuaternion(orientation)
        return {"position": np.asarray(position), "roll": np.rad2deg(euler[0]),
                "pitch": np.rad2deg(euler[1]), "yaw": np.rad2deg(euler[2])}
