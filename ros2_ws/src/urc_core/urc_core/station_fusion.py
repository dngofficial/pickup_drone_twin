import numpy as np
from .geometry import camera_mount, inv, matrix_pose, pose_matrix, wrap


class StationFusion:
    def __init__(self, initial, config):
        self.config = config
        self.estimate = np.asarray(initial, dtype=float).copy()
        self.applied = self.estimate.copy()
        self.cov = np.diag([.03**2, .03**2, .035**2, .02**2])
        self.last_fix = 0.
        self.accepted = 0
        self.rejected = 0
        self.last_observation = -1.
        self.relocalization = []

    def predict(self, dt):
        keys = ('correction_process_noise_xy_m2_per_s',) * 2 + ('correction_process_noise_z_m2_per_s', 'correction_process_noise_yaw_rad2_per_s')
        self.cov += np.diag([self.config[k] for k in keys]) * dt
        translation = self.estimate[:3] - self.applied[:3]
        limit = self.config['correction_speed_mps'] * dt
        self.applied[:3] += translation * min(1., limit / max(np.linalg.norm(translation), 1e-9))
        yaw_limit = self.config['correction_yaw_rate_radps'] * dt
        self.applied[3] = wrap(self.applied[3] + np.clip(wrap(self.estimate[3]-self.applied[3]), -yaw_limit, yaw_limit))

    def correct(self, observation, covariance, observed_at, now, stationary=False):
        if observed_at <= self.last_observation or not -.025 <= now-observed_at <= self.config['max_fix_age_s']:
            self.rejected += 1
            return False
        self.last_observation = observed_at
        innovation = np.asarray(observation) - self.estimate
        if not np.isfinite(innovation).all() or not np.isfinite(covariance).all() or np.min(np.linalg.eigvalsh(covariance)) <= 0:
            self.rejected += 1
            return False
        innovation[3] = wrap(innovation[3])
        s = self.cov + covariance
        if innovation @ np.linalg.solve(s, innovation) > self.config['correction_gate_mahalanobis2']:
            self.rejected += 1
            if stationary:
                self.relocalization = (self.relocalization + [observation])[-3:]
                if len(self.relocalization) == 3:
                    same = np.asarray(self.relocalization) - observation
                    if (np.max(np.linalg.norm(same[:, :3], axis=1)) < .15
                            and max(abs(wrap(x)) for x in same[:, 3]) < .08
                            and np.linalg.norm(innovation[:3]) < 1.5 and abs(innovation[3]) < .35):
                        self.estimate = np.asarray(observation).copy()
                        self.cov = covariance.copy()
                        self.last_fix, self.accepted = observed_at, self.accepted + 1
                        self.relocalization.clear()
                        return True
            else:
                self.relocalization.clear()
            return False
        gain = np.linalg.solve(s, self.cov).T
        self.estimate += gain @ innovation
        self.estimate[3] = wrap(self.estimate[3])
        identity = np.eye(4) - gain
        self.cov = identity @ self.cov @ identity.T + gain @ covariance @ gain.T
        self.last_fix = observed_at
        self.accepted += 1
        self.relocalization.clear()
        return True

    @staticmethod
    def tag_fix(surveyed, observed, local_pose, tilt):
        return matrix_pose(surveyed @ inv(observed) @ inv(camera_mount(tilt)) @ inv(pose_matrix(local_pose)))

    @property
    def horizontal_sigma(self):
        return float(np.sqrt(max(self.cov[0, 0], self.cov[1, 1])))

    def quality(self, now):
        age = now-self.last_fix
        if age > self.config['return_fix_age_s'] or self.horizontal_sigma > self.config['return_horizontal_sigma_m']:
            return 'RETURN'
        return 'LOW' if age > self.config['low_confidence_fix_age_s'] else 'GOOD'
