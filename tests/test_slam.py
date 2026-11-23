import numpy as np
from urc_core.slam import LidarSlam


def test_relative_barometer_instead_of_downward_range(config):
    slam=LidarSlam(config['localization']);rng=np.random.default_rng(29)
    points=rng.uniform([-3,-3,-1], [3,3,.9], (700,3)).tolist()
    def packet(rng): return dict(points=points,dt=.1,flow_xy=[0.,0.],gyro_z=0.,baro_height=.4,clearance_down=rng)
    assert slam.update(packet(.3),.1)
    assert abs(slam.pose[2]-.4)<.02
    assert slam.update(packet(4.),.2)
    assert abs(slam.pose[2]-.4)<.02
    assert not slam.update(packet(4.),.2)
