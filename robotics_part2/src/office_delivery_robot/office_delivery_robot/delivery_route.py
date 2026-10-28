"""Delivery stop ordering for the office robot (pure Python, no ROS imports).

AI-assisted. Rooms come from config/office_rooms.yaml (Gazebo world frame).
The map frame used by Nav2 is the world frame shifted by +1.0 m in x, the same
shift applied by localization.launch.py (map -> odom static transform).

All rooms open onto one straight corridor (world SC4 has no loops), so travel
cost between two stops is estimated along the corridor centreline y = 0.
"""

import itertools
import math
from collections import namedtuple

import yaml

Stop = namedtuple('Stop', 'room_id name x y yaw')

# Up to this many stops the best order is found by trying every permutation.
EXACT_LIMIT = 8


def load_layout(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def stops_from_layout(layout, room_ids):
    """Return (stops, rejected) for the requested room ids.

    Forbidden rooms and unknown ids are rejected with a reason string.
    Each stop is the room's approach point, facing into the room.
    """
    rooms = {room['id']: room for room in layout.get('rooms', [])}
    stops, rejected = [], []
    for room_id in room_ids:
        room = rooms.get(room_id)
        if room is None:
            rejected.append((room_id, 'unknown room'))
        elif room.get('access') != 'allowed':
            rejected.append((room_id, f"access is '{room.get('access')}'"))
        else:
            x, y = room['approach_point']
            yaw = -math.pi / 2.0 if y < 0.0 else math.pi / 2.0
            stops.append(Stop(room['id'], room['name'], float(x), float(y), yaw))
    return stops, rejected


def world_to_map(x, y, offset_x=1.0, offset_y=0.0):
    return x + offset_x, y + offset_y


def corridor_distance(p, q):
    """Estimated travel distance between (x, y) points via the corridor."""
    if abs(p[0] - q[0]) < 1e-9 and p[1] * q[1] > 0.0:
        return abs(p[1] - q[1])
    return abs(p[1]) + abs(p[0] - q[0]) + abs(q[1])


def route_length(start, order, dist=corridor_distance, return_to_start=False):
    points = [start] + [(s.x, s.y) for s in order]
    if return_to_start:
        points.append(start)
    return sum(dist(a, b) for a, b in zip(points, points[1:]))


def _nearest_neighbour(start, stops, dist):
    left, order, here = list(stops), [], start
    while left:
        nxt = min(left, key=lambda s: dist(here, (s.x, s.y)))
        left.remove(nxt)
        order.append(nxt)
        here = (nxt.x, nxt.y)
    return order


def _two_opt(start, order, dist, return_to_start):
    best = list(order)
    best_len = route_length(start, best, dist, return_to_start)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                trial = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                trial_len = route_length(start, trial, dist, return_to_start)
                if trial_len < best_len - 1e-9:
                    best, best_len, improved = trial, trial_len, True
    return best


def optimize_route(start, stops, dist=corridor_distance, return_to_start=False):
    """Order stops to minimise travel from start; returns (order, length_m).

    Exact (all permutations) for up to EXACT_LIMIT stops, otherwise
    nearest-neighbour followed by 2-opt.
    """
    stops = list(stops)
    if not stops:
        return [], 0.0
    if len(stops) <= EXACT_LIMIT:
        best = min(itertools.permutations(stops),
                   key=lambda order: route_length(start, order, dist, return_to_start))
        best = list(best)
    else:
        best = _two_opt(start, _nearest_neighbour(start, stops, dist), dist, return_to_start)
    return best, route_length(start, best, dist, return_to_start)
