"""Inflated local obstacle map and deterministic A* pursuit."""
import heapq
import math
import numpy as np


class Grid:
    def __init__(self, fence, resolution=.5):
        self.resolution = resolution
        self.origin = np.array([fence[0], fence[2]])
        self.size = (round((fence[1]-fence[0])/resolution), round((fence[3]-fence[2])/resolution))
        self.blocked = set()
        self.known = set()

    def cell(self, xy):
        return tuple(np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int))

    def center(self, cell):
        return self.origin + (np.asarray(cell)+.5)*self.resolution

    def update(self, points, radius):
        self.known = {self.cell(p[:2]) for p in points}
        self.blocked.clear()
        if not len(points):
            return
        # Low terrain is traversed above its surface. Tall returns and structures
        # are inflated for the XY transit plane. Approach descent has its own check.
        columns = {}
        for point in points:
            c = self.cell(point[:2])
            top, bottom = columns.get(c, (-math.inf, math.inf))
            columns[c] = max(top, point[2]), min(bottom, point[2])
        steps = math.ceil(radius/self.resolution)
        for (x, y), (top, bottom) in columns.items():
            if top > 1.20 and top-bottom > .35:
                for dx in range(-steps, steps+1):
                    for dy in range(-steps, steps+1):
                        if math.hypot(dx, dy)*self.resolution <= radius+.18:
                            self.blocked.add((x+dx,y+dy))

    def path(self, origin, goal):
        start, end = self.cell(origin), self.cell(goal)
        if start == end:
            return [np.asarray(goal)]

        def valid(cell):
            return (0 <= cell[0] < self.size[0] and 0 <= cell[1] < self.size[1]
                    and (cell in (start, end) or cell not in self.blocked))

        if not valid(start) or not valid(end):
            return []
        cost, previous = {start: 0.}, {}
        queue = [(math.dist(start, end), 0., start)]
        while queue:
            _, known_cost, here = heapq.heappop(queue)
            if known_cost > cost[here]:
                continue
            if here == end:
                cells = [here]
                while here != start:
                    here = previous[here]
                    cells.append(here)
                corners = [self.center(c) for c in reversed(cells[1:-1])] + [np.asarray(goal)]
                return self.smooth(np.asarray(origin), corners, valid)
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nx, ny = here[0]+dx, here[1]+dy
                nxt = (nx, ny)
                if not valid(nxt) or (dx and dy and (not valid((nx, here[1])) or not valid((here[0], ny)))):
                    continue
                new = known_cost + math.hypot(dx, dy) * (1.0 if nxt in self.known else 1.18)
                if new < cost.get(nxt, math.inf):
                    cost[nxt], previous[nxt] = new, here
                    heapq.heappush(queue, (new + math.dist(nxt, end), new, nxt))
        return []


    def smooth(self, origin, points, valid):
        def segment_clear(a, b):
            last = self.cell(a)
            divisions = max(1, math.ceil(np.linalg.norm(b-a) / (.24 * self.resolution)))
            for step in range(1, divisions+1):
                cell = self.cell(a+(b-a)*(step/divisions))
                if not valid(cell):
                    return False
                if last[0]!=cell[0] and last[1]!=cell[1]:
                    if not valid((last[0],cell[1])) or not valid((cell[0],last[1])):
                        return False
                last = cell
            return True

        route = []
        start = 0
        while start < len(points):
            candidate = len(points)-1
            while candidate>start and not segment_clear(origin,points[candidate]):
                candidate -= 1
            origin = points[candidate]
            route.append(origin)
            start = candidate+1
        return route


def lawnmower(bounds, spacing):
    xmin, xmax, ymin, ymax = bounds
    lanes = np.arange(ymin+spacing/2, ymax, spacing)
    result = []
    for i, y in enumerate(lanes):
        result.extend([(xmin+.3,y),(xmax-.3,y)] if i % 2 == 0 else [(xmax-.3,y),(xmin+.3,y)])
    return result


def select_bin(policy, kind, location, bins, counts, capacity, class_bins, selected):
    available = [name for name in bins if counts.get(name,0) < capacity]
    if policy == 'nearest':
        return min(available, key=lambda name: (np.linalg.norm(np.asarray(bins[name][:2])-location[:2]), name), default=None)
    preferred = selected if policy == 'ui_select' else class_bins.get(kind)
    return preferred if preferred in available else None
