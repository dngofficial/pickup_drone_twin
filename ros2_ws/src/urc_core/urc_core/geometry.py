"""REP-103 body FLU, optical right/down/forward, map XYZ+yaw."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = yaw_rotation(pose[3])
    transform[:3, 3] = pose[:3]
    return transform


def matrix_pose(matrix):
    return np.r_[matrix[:3, 3], math.atan2(matrix[1, 0], matrix[0, 0])]


def inv(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def transform_point(transform, point):
    return transform[:3, :3] @ point + transform[:3, 3]


def camera_mount(tilt):
    c, s = math.cos(tilt), math.sin(tilt)
    matrix = np.eye(4)
    matrix[:3, :3] = [[0, -s, c], [-1, 0, 0], [0, -c, -s]]
    matrix[:3, 3] = [.20, 0, -.15]
    return matrix


def pose_from_msg(message):
    transform = np.eye(4)
    transform[:3, 3] = [message['position'][axis] for axis in 'xyz']
    transform[:3, :3] = Rotation.from_quat([message['orientation'][axis] for axis in 'xyzw']).as_matrix()
    return transform


def vector(values):
    return dict(zip('xyz', map(float, values)))


def quaternion(matrix):
    return dict(zip('xyzw', Rotation.from_matrix(matrix).as_quat().tolist()))


def pose_msg(pose):
    return {'position': vector(pose[:3]), 'orientation': quaternion(yaw_rotation(pose[3]))}


def time_value(stamp):
    return int(stamp['sec']) + int(stamp['nanosec']) * 1e-9


def stamp(seconds):
    sec, nanos = divmod(round(seconds * 1e9), 10**9)
    return {'sec': sec, 'nanosec': nanos}


def header(seconds, frame):
    return {'stamp': stamp(seconds), 'frame_id': frame}
