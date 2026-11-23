from dataclasses import dataclass
import numpy as np
from scipy.optimize import linear_sum_assignment

NEW, CONFIRMED, TARGETED, PICKED, LOST, DELIVERED = range(6)


@dataclass
class Track:
    id: str
    kind: str
    position: np.ndarray
    seen: float
    confidence: float
    hits: int = 1
    status: int = NEW
    attempts: int = 0
    blacklisted: bool = False
    frame_first: int = 0
    observation: np.ndarray | None = None
    defer_until: float = 0.


class Tracker:
    def __init__(self, config):
        self.config = config
        self.tracks = {}
        self.serial = 0
        self.frame = 0

    def update(self, observations, now):
        self.frame += 1
        live = [x for x in self.tracks.values() if x.status not in (PICKED, DELIVERED)]
        gate = self.config['track_association_gate_m']
        used = set()
        if live and observations:
            costs = np.full((len(live), len(observations)), 1e6)
            for i, track in enumerate(live):
                for j, (kind, _, xyz) in enumerate(observations):
                    distance = np.linalg.norm(track.position-xyz)
                    if track.kind == kind and distance <= gate:
                        costs[i, j] = distance
            rows, cols = linear_sum_assignment(costs)
            for i, j in zip(rows, cols):
                if costs[i, j] > gate:
                    continue
                used.add(j)
                track = live[i]
                _, score, xyz = observations[j]
                if track.status == NEW and self.frame-track.frame_first >= self.config['track_confirmation_window']:
                    track.hits, track.frame_first = 0, self.frame
                track.position += .55 * (xyz-track.position)
                track.observation = xyz.copy()
                track.seen, track.confidence = now, score
                track.hits += 1
                if track.status in (NEW, LOST) and track.hits >= self.config['track_confirmation_hits']:
                    track.status = CONFIRMED
        for j, (kind, score, xyz) in enumerate(observations):
            if j not in used:
                if any(x.status == DELIVERED and x.kind == kind and np.linalg.norm(x.position-xyz) < .34 for x in self.tracks.values()):
                    continue
                self.serial += 1
                key = f'T{self.serial:03d}'
                self.tracks[key] = Track(key, kind, xyz.copy(), now, score, frame_first=self.frame, observation=xyz.copy())
        for track in live:
            if now-track.seen > self.config['track_lost_timeout_s']:
                track.status, track.hits = LOST, 0
        return self.tracks
