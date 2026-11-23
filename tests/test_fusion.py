import numpy as np
from urc_core.geometry import camera_mount, pose_matrix, inv
from urc_core.station_fusion import StationFusion


def test_tag_transform_and_nonidentity_gate(config):
    setting=config['localization']
    fix=np.array([.08,-.02,.65,.02]); local=np.array([.65,.22,2.8,.07])
    target=pose_matrix([2.5,1.1,.62,.1]); angle=1.5
    detection=inv(pose_matrix(fix)@pose_matrix(local)@camera_mount(angle))@target
    np.testing.assert_allclose(StationFusion.tag_fix(target,detection,local,angle),fix,atol=1e-9)
    fuse=StationFusion([0.,0.,.65,0.],setting); noise=np.eye(4)*.0025
    assert fuse.correct(fix,noise,.1,.084)
    assert not fuse.correct([50.,0.,.65,0.],noise,.2,.2)
    assert not fuse.correct(fix,noise,.1,.3)
    fuse.predict(.5)
    assert fuse.applied[0]>0 and fuse.applied[2]>.60
    assert fuse.quality(91.)=='RETURN'


def test_consecutive_relocalization_checks_consistency(config):
    f=StationFusion([0.,0.,.65,0.],config['localization'])
    noise=np.eye(4)*.001
    for i,x in enumerate((.6,.62,.61),1):
        assert f.correct(np.array([x,0.,.65,0.]),noise,i,i,stationary=True)==(i==3)
