"""Four-DOF local LiDAR registration adapted from the MIT warehouse twin."""
import numpy as np
from scipy.spatial import cKDTree
from .geometry import wrap, yaw_rotation


def voxel(points, size):
    _, first = np.unique(np.floor(points / size).astype(np.int32), axis=0, return_index=True)
    return points[np.sort(first)]


class LidarSlam:
    def __init__(self, config):
        self.config = config
        self.pose = np.zeros(4)
        self.velocity = np.zeros(3)
        self.points = np.empty((0, 3))
        self.tree = None
        self.normals = None
        self.last_keyframe = None
        self.barometer = None
        self.rmse = 0.
        self.inliers = 0.
        self.valid = False
        self.registered = False
        self.latest = -1.

    def update(self, packet, now):
        cloud = np.asarray(packet['points'], dtype=float).reshape(-1, 3)
        dt = float(packet['dt'])
        flow = np.asarray(packet['flow_xy'], dtype=float)
        gyro = float(packet['gyro_z'])
        height = float(packet['baro_height'])
        if (not .005 <= dt <= 2.0 or now <= self.latest or len(cloud) < 60 or len(cloud) > 20000
                or flow.shape != (2,) or not np.isfinite([gyro, height, *flow]).all()
                or np.linalg.norm(flow) > 5. or abs(gyro) > 3.):
            return False
        points = cloud[np.isfinite(cloud).all(axis=1)]
        if len(points) < 60:
            return False
        points = points[::max(1, len(points)//550)][:1000]
        prior = self.pose.copy()
        prior[:2] += (yaw_rotation(prior[3] + .5 * gyro * dt) @ np.r_[flow, 0.])[:2] * dt
        prior[3] = wrap(prior[3] + gyro * dt)
        self.barometer = height if self.barometer is None else .22 * height + .78 * self.barometer
        prior[2] = .62 * height + .38 * self.barometer
        estimate = prior.copy()
        self.registered = False
        if self.tree is not None:
            estimate = self.register(points, prior)
        raw_speed = (estimate[:3] - self.pose[:3]) / dt
        self.velocity = .75 * self.velocity + .25 * raw_speed
        self.pose = estimate
        self.latest = now
        self.valid = True
        if (self.last_keyframe is None or np.linalg.norm(estimate[:3]-self.last_keyframe[:3]) > .38
                or abs(wrap(estimate[3]-self.last_keyframe[3])) > .27):
            if self.tree is None or self.registered or self.inliers >= .25:
                self.add_keyframe(points)
        return True

    def register(self, points, prior):
        guess = prior.copy()
        for _ in range(3):
            rotated = points @ yaw_rotation(guess[3]).T
            world = rotated + guess[:3]
            distance, index = self.tree.query(world)
            normals = self.normals[index]
            residual = np.einsum('ij,ij->i', normals, world-self.points[index])
            keep = (distance < .65) & (np.abs(residual) < .13)
            self.inliers = float(keep.mean())
            if keep.sum() < 45 or self.inliers < .22:
                return prior
            normals, rotated, residual = normals[keep], rotated[keep], residual[keep]
            self.rmse = float(np.sqrt(np.mean(residual**2)))
            yaw_jacobian = -normals[:, 0] * rotated[:, 1] + normals[:, 1] * rotated[:, 0]
            jacobian = np.c_[normals, yaw_jacobian]
            weights = np.sqrt(np.minimum(1., .04 / np.maximum(np.abs(residual), 1e-6)))
            a = jacobian * weights[:, None]
            # Body velocity and gyro constrain poorly observed horizontal terrain.
            regularizer = np.array([110., 110., 28., 500.]) ** 2
            delta = np.linalg.solve(a.T @ a + np.diag(regularizer),
                                    -a.T @ (residual * weights) + regularizer * (prior - guess))
            guess += np.clip(delta, [-.03, -.03, -.06, -.02], [.03, .03, .06, .02])
            if np.linalg.norm(delta) < .0005:
                break
        self.registered = True
        guess[3] = wrap(guess[3])
        return guess

    def add_keyframe(self, points):
        world = points @ yaw_rotation(self.pose[3]).T + self.pose[:3]
        limit = min(16000, int(self.config['max_map_points']))
        self.points = voxel(np.r_[self.points, world], self.config['voxel_size_m'])[-limit:]
        self.tree = cKDTree(self.points)
        _, near = self.tree.query(self.points, k=min(8, len(self.points)))
        patches = self.points[near]
        centered = patches - patches.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered)
        _, bases = np.linalg.eigh(covariance)
        self.normals = bases[:, :, 0]
        self.last_keyframe = self.pose.copy()
