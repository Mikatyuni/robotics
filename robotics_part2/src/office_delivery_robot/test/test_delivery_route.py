import math
import os

from office_delivery_robot.delivery_route import (
    Stop, corridor_distance, load_layout, optimize_route, route_length,
    stops_from_layout, world_to_map)

ROOMS_YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'office_rooms.yaml')


def stop(name, x, y):
    return Stop(name, name, x, y, 0.0)


def test_corridor_distance_symmetric_and_not_shorter_than_straight_line():
    pts = [(0.0, -2.4), (4.7, -2.4), (-9.9, 2.4), (11.8, -2.4), (0.0, 0.0)]
    for a in pts:
        for b in pts:
            assert math.isclose(corridor_distance(a, b), corridor_distance(b, a))
            assert corridor_distance(a, b) >= math.dist(a, b) - 1e-9


def test_exact_route_beats_given_order():
    start = (0.0, 0.0)
    stops = [stop('far', 10.0, -2.4), stop('near', 1.0, -2.4), stop('mid', 5.0, -2.4)]
    order, length = optimize_route(start, stops)
    assert [s.room_id for s in order] == ['near', 'mid', 'far']
    assert length < route_length(start, stops)


def test_return_to_start_is_counted():
    start = (0.0, 0.0)
    stops = [stop('a', 4.0, -2.4), stop('b', 8.0, -2.4)]
    _, open_len = optimize_route(start, stops)
    _, closed_len = optimize_route(start, stops, return_to_start=True)
    assert closed_len > open_len


def test_large_route_visits_every_stop_once_and_is_no_worse_than_naive():
    start = (0.0, 0.0)
    xs = [-11, 9, -3, 7, -8, 2, 12, -5, 4, -1, 10]
    stops = [stop(f's{i}', float(x), -2.4 if i % 2 else 2.4) for i, x in enumerate(xs)]
    order, length = optimize_route(start, stops)
    assert sorted(s.room_id for s in order) == sorted(s.room_id for s in stops)
    assert length <= route_length(start, stops)


def test_empty_route():
    assert optimize_route((0.0, 0.0), []) == ([], 0.0)


def test_layout_rejects_forbidden_and_unknown_rooms():
    layout = load_layout(ROOMS_YAML)
    stops, rejected = stops_from_layout(layout, ['hr', 'bathroom', 'nowhere', 'it'])
    assert [s.room_id for s in stops] == ['hr', 'it']
    assert {r[0] for r in rejected} == {'bathroom', 'nowhere'}
    assert stops[0].yaw < 0.0  # HR is south of the corridor, faces -y


def test_world_to_map_shift_matches_localization_launch():
    assert world_to_map(-1.0, 0.0) == (0.0, 0.0)
