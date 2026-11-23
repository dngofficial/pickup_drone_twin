import numpy as np


def backproject(u, v, optical_depth, intrinsics):
    if not np.isfinite([u, v, optical_depth]).all() or not .035 <= optical_depth <= 60.:
        raise ValueError('invalid optical sample')
    fx, cx, fy, cy = [intrinsics[i] for i in (0, 2, 4, 5)]
    return np.array([(u-cx)*optical_depth/fx, (v-cy)*optical_depth/fy, optical_depth])


def foreground_point(image, box):
    """Find a spatially supported near-depth cluster in the inner box."""
    x, y, w, h = box
    y0, y1 = max(0, int(y-.35*h)), min(image.shape[0], int(y+.35*h)+1)
    x0, x1 = max(0, int(x-.35*w)), min(image.shape[1], int(x+.35*w)+1)
    if y1 <= y0 or x1 <= x0:
        return None
    crop = image[y0:y1, x0:x1].astype(float) * .001
    valid = (crop >= .035) & (crop <= 60.)
    if valid.sum() < 3:
        return None
    near = np.quantile(crop[valid], .12, method='lower')
    rows, cols = np.where(valid & (np.abs(crop-near) < .07))
    if len(rows) < 3:
        return None
    return float(np.median(cols)+x0), float(np.median(rows)+y0), float(np.median(crop[rows, cols]))


def ground_intersection(origin, ray, normal, offset):
    denom = float(np.dot(ray, normal))
    if abs(denom) < .06:
        return None
    distance = -(float(np.dot(origin, normal)) + offset) / denom
    return origin + distance * ray if 0 < distance < 60 else None
