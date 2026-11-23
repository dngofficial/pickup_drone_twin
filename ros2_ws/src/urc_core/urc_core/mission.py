from dataclasses import dataclass, field
import math
import numpy as np
from .geometry import wrap, yaw_rotation
from .planning import Grid, lawnmower, select_bin
from .tracker import CONFIRMED, TARGETED, PICKED, LOST, DELIVERED


@dataclass
class Command:
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.
    grip: str = ''
    request: str = ''
    tilt: float = math.pi/2


class Mission:
    def __init__(self, settings, seed_count=None):
        self.config = settings['mission']
        self.safety = settings['safety']
        site = settings['scene']
        self.expected = seed_count if seed_count is not None else site['bottles']+site['hammers']
        self.home = np.array(site['initial_base_map_xyz_yaw'][:3])
        self.bins = {k: np.array(site[k+'_map_xyz']) for k in ('bin_a','bin_b')}
        self.rim, self.capacity = site['bin_rim_height_m'], site['bin_capacity']
        self.grid = Grid(self.safety['geofence_xy'], self.config['coverage_cell_m'])
        self.covered = set()
        self.route = lawnmower(self.config['search_bounds_xy'], self.config['lane_spacing_m'])
        self.lane = 0
        self.revisit_planned = False
        self.policy, self.selected_bin = self.config['bin_policy'], self.config['selected_bin']
        self.state, self.reason = 'IDLE', 'ready'
        self.paused = False
        self.estop = False
        self.manual = False
        self.resume_state = 'SEARCH'
        self.entered = 0.
        self.active = False
        self.outcome = None
        self.mission_id = ''
        self.now = 0.
        self.energy = self.safety['battery_capacity_wh']
        self.target_id = ''
        self.destination = None
        self.payload_token = ''
        self.pending = ''
        self.serial = 0
        self.last_goal = None
        self.path = []
        self.grid_revision = 0
        self.path_revision = -1
        self.deliveries = {}
        self.grasps = {}
        self.counts = {'bin_a': 0, 'bin_b': 0}
        self.confirmed_at = {}
        self.local_loss = None
        self.reacquire_baseline = 0
        self.return_reason = ''
        self.return_after_drop = False
        self.return_via = False
        self.initial_station_pass = False
        self.motion_ready = None

    @property
    def battery(self):
        return self.energy / self.safety['battery_capacity_wh']

    def transition(self, state, reason=''):
        self.state, self.reason, self.entered = state, reason, self.now
        self.last_goal, self.path, self.pending = None, [], ''
        self.motion_ready = None

    def start(self, mission_id):
        if self.active or self.estop or self.outcome is not None or not mission_id:
            return False
        self.active, self.mission_id, self.paused, self.manual = True, mission_id, False, False
        self.transition('TAKEOFF')
        return True

    def pause(self):
        self.paused = True

    def resume(self):
        if self.estop:
            return False
        self.paused, self.manual = False, False
        if self.state == 'E_STOP':
            self.transition(self.resume_state)
        return True

    def emergency_stop(self):
        if not self.estop:
            self.resume_state = self.state
        self.estop, self.paused = True, True
        self.transition('E_STOP', 'operator latch')

    def clear_estop(self):
        self.estop = False
        return True

    def request_return(self, why):
        self.return_reason = why
        self.return_via = False
        self.transition('RETURN_HOME', why)

    def finish(self, success, reason):
        self.outcome = success
        self.active = False
        self.transition('IDLE' if success else 'FAILED', reason)

    def select_target(self, tracks, track_id):
        track = tracks.get(track_id)
        if (not self.active or self.state not in ('SEARCH','CONFIRM_TARGET','APPROACH') or track is None
                or track.blacklisted or track.status not in (CONFIRMED,TARGETED) or self.now-track.seen > .85):
            return False
        previous = tracks.get(self.target_id)
        if previous and previous.id != track.id:
            previous.status = CONFIRMED
        self.target_id, track.status = track.id, TARGETED
        self.confirmed_at.setdefault(track.id, self.now)
        self.transition('CONFIRM_TARGET')
        return True

    def deliver(self, event, tracks):
        if event['event_id'] in self.deliveries or event['payload_token'] not in self.grasps:
            return
        track_id, kind, expected_bin, grabbed = self.grasps[event['payload_token']]
        self.deliveries[event['event_id']] = (event['bin_id'],kind,expected_bin,grabbed,self.now,track_id)
        self.counts[event['bin_id']] += 1
        if track_id in tracks:
            track = tracks[track_id]
            track.status = DELIVERED

    def abort_target(self, tracks, reason, missed=False):
        target = tracks.get(self.target_id)
        if target:
            target.defer_until = self.now + (5. if missed else 16.)
            if missed:
                target.attempts += 1
            target.blacklisted = target.attempts >= self.config['max_grasp_attempts']
            target.status = LOST if self.now-target.seen > .6 else CONFIRMED
        self.transition('RESUME_SEARCH', reason)

    def goal(self, xyz, pose, max_xy, max_z=.45):
        world = np.asarray(xyz) - pose[:3]
        xy = world[:2] * min(1.25, max_xy / max(np.linalg.norm(world[:2]), 1e-9))
        velocity = yaw_rotation(pose[3]).T @ np.r_[xy, np.clip(world[2] * 1.4, -max_z, max_z)]
        return Command(velocity, float(np.clip(-wrap(pose[3]), -.55,.55)))

    def pursue(self, goal, pose, max_xy, max_z=.45):
        xy = np.asarray(goal[:2])
        if self.last_goal is None or np.linalg.norm(self.last_goal-xy) > .3 or self.grid_revision != self.path_revision:
            self.path = self.grid.path(pose[:2], xy)
            self.last_goal = xy.copy()
            self.path_revision = self.grid_revision
        while len(self.path)>1 and np.linalg.norm(self.path[0]-pose[:2]) < .31:
            self.path.pop(0)
        if not self.path:
            return self.goal([*pose[:2],goal[2]], pose, 0., max_z)
        waypoint = self.path[0] if len(self.path)>1 else xy
        return self.goal([*waypoint,goal[2]], pose, max_xy, max_z)

    def grasp_command(self, action, retry=False):
        if not self.pending or retry:
            self.serial += 1
            self.pending = f'mission-{self.serial}'
        return Command(grip=action, request=self.pending)

    def coverage(self, pose):
        low_x, high_x, low_y, high_y = self.config['search_bounds_xy']
        r = self.config['coverage_cell_m']
        return min(1., len(self.covered) / max(1, round((high_x-low_x)/r)*round((high_y-low_y)/r)))

    def mark_coverage(self, pose, clearance):
        if not 1.6 <= clearance <= 4.2:
            return
        a,b,c,d = self.config['search_bounds_xy']
        center = self.grid.cell(pose[:2])
        for dx in range(-3,4):
            for dy in range(-5,6):
                cell=(center[0]+dx,center[1]+dy)
                x,y = self.grid.center(cell)
                if a <= x < b and c <= y < d and ((x-pose[0])/(clearance*.56))**2+((y-pose[1])/(clearance*.90))**2 < 1:
                    self.covered.add(cell)

    def step(self, now, dt, pose, speed, clearance, quality, valid, tracks, gripper, accepted_fixes, horizontal_sigma=0.):
        self.now = now
        payload = float(gripper.get('payload_mass_kg',0.))
        if self.active:
            rate = self.safety['battery_hover_w'] + speed*self.safety['battery_translation_w_per_mps'] + payload*self.safety['battery_payload_w_per_kg']
            self.energy = max(0., self.energy-rate*dt/3600)
        if self.paused or self.estop or self.manual:
            self.entered += dt
            if self.motion_ready is not None:
                self.motion_ready += dt
            return Command()
        if not self.active:
            return Command()
        if not valid or not math.isfinite(clearance) or horizontal_sigma > self.safety.get('fail_horizontal_sigma_m',2.):
            self.local_loss = now if self.local_loss is None else self.local_loss
            if now-self.local_loss > 3.0:
                self.finish(False,'localization or clearance unavailable')
            return Command()
        self.local_loss = None
        target = tracks.get(self.target_id)
        elapsed = now-self.entered
        agl = clearance + .18
        transit = self.config['transit_speed_mps'] / (1+self.safety['payload_speed_coefficient_per_kg']*payload)
        home_distance = np.linalg.norm(pose[:2]-self.home[:2])
        reserve_wh = (home_distance / max(transit,.1) + 12)*self.safety['battery_hover_w']/3600*self.safety['rtl_energy_margin_factor']
        if self.battery < self.safety['battery_critical_fraction'] and self.state != 'LAND':
            self.return_reason = 'critical battery'; self.transition('LAND','local landing')
        elif self.state not in ('RETURN_HOME','LAND','DROP'):
            if self.battery < self.safety['battery_rtl_fraction'] or self.energy < reserve_wh:
                destination = self.bins.get(self.destination) if self.state=='TRANSPORT' and gripper.get('attached') else None
                if destination is not None:
                    delivery_time = np.linalg.norm(destination[:2]-pose[:2])/max(transit,.1) + 12.
                    return_time = np.linalg.norm(destination[:2]-self.home[:2])/max(self.config['transit_speed_mps'],.1) + 12.
                    required = self.safety['rtl_energy_margin_factor']*(delivery_time*rate+return_time*self.safety['battery_hover_w'])/3600
                    if self.energy >= required and quality!='RETURN':
                        self.return_after_drop = True
                    else:
                        self.request_return('battery reserve')
                else:
                    self.request_return('battery reserve')
            elif quality == 'RETURN':
                self.reacquire_baseline = accepted_fixes
                self.request_return('station fix expired')
        elapsed = now-self.entered
        if self.state == 'TAKEOFF':
            if agl >= 3.12:
                self.transition('SEARCH')
            return self.goal([*self.home[:2], pose[2]+self.config['search_altitude_agl_m']+.24-agl],pose,.22)
        if self.state == 'SEARCH':
            if not self.initial_station_pass:
                reacquire=np.array([-2.0,0.])
                if np.linalg.norm(reacquire-pose[:2]) < .18 and pose[2] > 3.55:
                    self.initial_station_pass=True
                else:
                    return self.pursue([*reacquire,3.75],pose,.58)
            if self.lane >= len(self.route):
                if len(self.deliveries) < self.expected and not self.revisit_planned:
                    self.revisit_planned = True
                    remaining = [t for t in tracks.values() if t.status != DELIVERED and not t.blacklisted]
                    remaining.sort(key=lambda t: np.linalg.norm(t.position[:2]-pose[:2]))
                    self.route.extend(tuple(t.position[:2]) for t in remaining
                                      if all(np.linalg.norm(t.position[:2]-b[:2]) > 1. for b in self.bins.values()))
                if self.lane >= len(self.route):
                    self.request_return('coverage complete')
                    return Command()
            available = [t for t in tracks.values() if (t.status==CONFIRMED and not t.blacklisted and now>=t.defer_until and now-t.seen < .65
                         and min(np.linalg.norm(t.position[:2]-b[:2]) for b in self.bins.values()) > 1.0)]
            if available and len(self.deliveries) < self.expected:
                nearest = min(available,key=lambda t:np.linalg.norm(t.position[:2]-pose[:2]))
                self.select_target(tracks,nearest.id)
                return Command()
            waypoint = self.route[self.lane]
            if np.linalg.norm(np.asarray(waypoint)-pose[:2]) < .32:
                self.lane += 1
            return self.pursue([*waypoint,pose[2]+np.clip(3.-agl,-.3,.3)],pose,transit)
        if self.state == 'CONFIRM_TARGET':
            if target is not None and now-target.seen < .6 and elapsed >= .3:
                self.transition('APPROACH')
            elif elapsed > 1.4:
                self.abort_target(tracks,'target confirmation expired')
            return Command()
        if self.state in ('APPROACH','DESCEND'):
            timeout = self.config['max_state_duration_s']
            if target is None or now-target.seen > (3.2 if self.state=='APPROACH' else 1.1) or elapsed > timeout:
                self.abort_target(tracks,'target observation or path expired')
                return Command()
            aim = target.position
            fresh = target.observation if now-target.seen < .5 and target.observation is not None else aim
            distance = np.linalg.norm(aim[:2]-pose[:2])
            if self.state == 'APPROACH':
                # The front offset means the object may briefly leave the image near grasp height.
                if distance < .22 and now-target.seen < .5:
                    self.transition('DESCEND')
                return self.pursue([*aim[:2],aim[2]+2.6],pose,min(.60,transit))
            desired_height = fresh[2]+.525
            if np.linalg.norm(fresh[:2]-pose[:2]) < .052 and (abs(pose[2]-desired_height) < .043):
                if self.motion_ready is None:
                    self.motion_ready = now
                if now-self.motion_ready >= .34 and speed < .10:
                    self.transition('GRASP')
                return Command()
            self.motion_ready = None
            return self.goal([*fresh[:2],desired_height], pose, .30, self.config['descent_speed_mps'])
        if self.state == 'GRASP':
            self.transition('VERIFY')
            return self.grasp_command('close')
        if self.state == 'VERIFY':
            responded = gripper.get('request_id') == self.pending
            if responded and gripper.get('attached') and elapsed >= self.config['attachment_verify_s']:
                token = gripper['payload_token']
                self.payload_token = token
                self.destination = select_bin(self.policy,target.kind,pose,self.bins,self.counts,self.capacity,self.config['class_bins'],self.selected_bin)
                if target:
                    target.status = PICKED
                self.grasps[token] = (self.target_id,target.kind,self.destination,self.confirmed_at.get(self.target_id,now))
                self.transition('ASCEND')
                return Command()
            if elapsed > 1.3:
                self.abort_target(tracks,'missed attachment', missed=True)
                return self.grasp_command('open')
            return self.grasp_command('close')
        if self.state in ('ASCEND','TRANSPORT') and not gripper.get('attached'):
            self.abort_target(tracks,'payload lost', missed=True)
            return Command()
        if self.state == 'ASCEND':
            if agl > 2.40:
                if self.destination is None:
                    self.destination = select_bin(self.policy,target.kind,pose,self.bins,self.counts,self.capacity,self.config['class_bins'],self.selected_bin)
                    if self.destination is None:
                        if elapsed > self.config['bin_policy_ui_timeout_s']:
                            self.pause(); self.reason = 'select available bin'
                        return Command()
                    token = self.payload_token
                    tid, kind, _, picked = self.grasps[token]
                    self.grasps[token] = tid, kind, self.destination, picked
                self.transition('TRANSPORT')
            return self.goal([*pose[:2],pose[2]+2.5-agl],pose,0.)
        if self.state == 'TRANSPORT':
            destination = self.bins[self.destination]
            distance = np.linalg.norm(destination[:2]-pose[:2])
            if distance < .55:
                height = destination[2]+self.rim+self.config['drop_height_above_rim_m']+.55
                if distance < .12 and abs(pose[2]-height)<.10 and speed<.19:
                    self.transition('DROP')
                    return Command()
                return self.goal([*destination[:2],height],pose,.32,.24)
            altitude = self.config['transport_altitude_agl_m']
            # The mast is 2.1 m tall. A loaded gripper hangs about 0.6 m
            # below the body and needs clearance across the station sector.
            if pose[0] < 3.5:
                altitude = max(altitude,3.25)
                if pose[2]<3.05 and np.linalg.norm(pose[:2]-[-.64,1.38])<2.7:
                    return self.goal([*pose[:2],3.25],pose,0.)
            return self.pursue([*destination[:2],max(altitude,pose[2]+altitude-agl)],pose,transit)
        if self.state == 'DROP':
            if self.motion_ready is None:
                self.motion_ready = now
            if elapsed < .32:
                return Command()
            if any(event[5]==self.target_id for event in self.deliveries.values()):
                self.target_id = ''
                if self.return_after_drop:
                    self.return_after_drop = False
                    self.request_return('battery reserve')
                else:
                    self.transition('RESUME_SEARCH','delivery confirmed')
                return Command()
            if elapsed > 4.0:
                self.abort_target(tracks,'release did not settle in bin',missed=True); return Command()
            return self.grasp_command('open')
        if self.state == 'RESUME_SEARCH':
            if agl > 2.78:
                self.target_id = ''; self.transition('SEARCH')
            return self.goal([*pose[:2],pose[2]+3.0-agl],pose,0.)
        if self.state == 'RETURN_HOME':
            if self.return_reason == 'station fix expired' and accepted_fixes-self.reacquire_baseline >= 3 and quality=='GOOD' and home_distance<4.5:
                self.transition('TRANSPORT' if gripper.get('attached') and self.destination else 'RESUME_SEARCH','station reacquired'); return Command()
            if self.return_reason == 'station fix expired' and not self.return_via:
                if np.linalg.norm(pose[:2]-[-2.0,0.]) < .19:
                    self.return_via = True
                else:
                    return self.pursue([-2.0,0.,3.75],pose,max(.28,transit))
            if home_distance < .19:
                self.transition('LAND',self.return_reason); return Command()
            return self.pursue([*self.home[:2],2.8],pose,max(.26,transit))
        if self.state == 'LAND':
            if (agl < .66 and speed < .16) or elapsed > 20.:
                landed = agl < .70
                self.finish(landed and (self.return_reason=='coverage complete' or self.return_reason=='battery reserve' and len(self.deliveries)>=self.expected), self.return_reason or 'landed')
                return Command()
            return self.goal([*pose[:2],pose[2]+.59-agl],pose,0.,.16)
        return Command()
