import numpy as np
from urc_core.mission import Mission
from urc_core.runtime import Twin, SENSORS
from urc_core.tracker import Track, CONFIRMED, DELIVERED


def tick(m,t,clearance=.47,quality='GOOD',valid=True):
    return m.step(t,.05,np.array([0.,0.,.65,0.]),0.,clearance,quality,valid,{}, {},0)


def test_emergency_stop_is_latched_and_requires_resume(config):
    m=Mission(config);assert m.start('test')
    assert tick(m,.05).velocity[2]>0
    m.emergency_stop();assert m.state=='E_STOP' and not m.resume()
    assert not tick(m,.1).velocity.any()
    m.clear_estop();assert m.paused and not tick(m,.15).velocity.any()
    assert m.resume() and m.state=='TAKEOFF'


def test_low_reserve_and_no_pose(config):
    m=Mission(config);m.start('test');m.energy=22.
    tick(m,.05);assert m.state in ('RETURN_HOME','LAND')
    m=Mission(config);m.start('test')
    tick(m,1.,valid=False);tick(m,4.2,valid=False)
    assert m.state=='FAILED' and not m.active


def test_count_only_verified_settlement_and_deduplicate(config):
    m=Mission(config)
    track=Track('T1','bottle',np.zeros(3),0.,1.,status=CONFIRMED)
    event={'event_id':'1-token','payload_token':'token','bin_id':'bin_a'}
    m.deliver(event,{'T1':track}); assert not m.deliveries
    m.grasps['token']=('T1','bottle','bin_a',0.)
    m.deliver(event,{'T1':track});m.deliver(event,{'T1':track})
    assert len(m.deliveries)==1 and track.status==DELIVERED and m.counts['bin_a']==1


def test_core_refuses_truth_and_missing_yolo_weights(config):
    twin=Twin(config,lambda *_: None)
    assert not any('ground_truth' in topic or '/evaluation' in topic for topic in SENSORS)
    reply=twin.rpc('set_detector',{'mode':'yolo'},epoch=1)
    assert not reply['ok'] and twin.wanted_detector=='oracle'
    assert not twin.rpc('estop',{},epoch=0)['ok']
    assert twin.rpc('estop', {}, epoch=1)['ok']
    assert not twin.rpc('reset_scene', {'seed': 42}, epoch=1)['ok']


def test_pause_preserves_state_timer(config):
    m=Mission(config);m.start('test');m.pause()
    for i in range(20): tick(m,(i+1)*.05)
    assert abs(m.entered-1.)<1e-9 and m.state=='TAKEOFF'
    assert m.resume()
    assert tick(m,1.05).velocity[2]>0


def test_resume_requires_clear_and_preserves_the_mission(config):
    sent = []
    twin = Twin(config, lambda topic, message: sent.append((topic, message)))
    assert twin.rpc('start_mission', {'mission_id': 'resume-test'}, epoch=1)['ok']
    twin.mission.transition('TRANSPORT')
    twin.rpc('estop', {}, epoch=1)
    sent.clear()
    denied = twin.rpc('resume_mission', {}, epoch=1)
    assert not denied['ok'] and 'Clear' in denied['error']
    assert not sent and twin.mission.estop and twin.mission.paused
    assert twin.rpc('clear_estop', {}, epoch=1)['ok']
    assert twin.mission.paused
    assert twin.rpc('resume_mission', {}, epoch=1)['ok']
    assert twin.mission.state == 'TRANSPORT' and twin.mission.active
    assert not twin.mission.paused and not twin.mission.estop
    assert any(topic == '/sim/control/request' and 'resume' in message['data'] for topic, message in sent)


def test_telemetry_reports_missing_yolo_weights_and_rejection_keeps_oracle(config, tmp_path):
    import json
    sent = []
    twin = Twin(config, lambda topic, message: sent.append((topic, message)), model=str(tmp_path/'missing.onnx'))
    twin.telemetry(twin.pose, 'GOOD', False)
    telemetry = json.loads(next(message['data'] for topic, message in sent if topic == '/ui/telemetry'))
    assert not telemetry['yolo_available']
    reply = twin.rpc('set_detector', {'mode': 'yolo'}, epoch=1)
    assert not reply['ok'] and 'YOLO model missing' in reply['error']
    assert twin.wanted_detector == 'oracle'


def test_low_battery_finishes_safe_nearby_delivery(config):
    m=Mission(config);m.start('test')
    target=Track('T1','bottle',np.array([4.,0.,0.]),0.,1.,status=CONFIRMED)
    m.target_id=target.id;m.destination='bin_a';m.transition('TRANSPORT');m.energy=24.
    close_to_bin=np.array([-2.,1.5,3.2,0.])
    m.step(1.,.05,close_to_bin,0.,3.,'GOOD',True,{'T1':target},{'attached':True,'payload_mass_kg':.15},3)
    assert m.state=='TRANSPORT' and m.return_after_drop
    m.step(1.1,.05,close_to_bin,0.,3.,'RETURN',True,{'T1':target},{'attached':True,'payload_mass_kg':.15},3)
    assert m.state=='RETURN_HOME'
