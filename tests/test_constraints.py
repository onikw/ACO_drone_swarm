"""Unit tests for the constraint validator with deliberately broken solutions."""
import pytest

from graph_generator import generate_grid_graph
from solution import Solution
from constraints import validate_solution


@pytest.fixture
def graph():
    return generate_grid_graph(rows=2, cols=3, seed=0)


@pytest.fixture
def base_battery():
    return [60.0, 60.0]


def _make_valid_solution(graph) -> Solution:
    """Minimal hand-crafted valid solution for a 2x3 grid (nodes 0-5, base=0)."""
    # Drone 0: 0 → 1 → 2 → 0
    # Drone 1: 0 → 3 → 4 → 5 → 0
    # This may not be globally optimal but is structurally valid.
    g = graph
    base = g.base_node  # 0

    routes = {
        0: [0, 1, 2, 0],
        1: [0, 3, 4, 5, 0],
    }

    def build_sched(route):
        sched = []
        t = 0.0
        prev = base
        for v in route[1:]:
            if v == base:
                break
            tt = g.travel_time(prev, v)
            arrive = t + tt
            depart = arrive + g.search_time(v)
            sched.append((v, arrive, depart))
            t = depart
            prev = v
        return sched

    sched0 = build_sched(routes[0])
    sched1 = build_sched(routes[1])

    def makespan_from(sched, route):
        if not sched:
            return 0.0
        last_v, _, dep = sched[-1]
        return dep + g.shortest_path_length(last_v, base)

    ms = max(makespan_from(sched0, routes[0]), makespan_from(sched1, routes[1]))

    battery_usage = {}
    for d, route in routes.items():
        # Use shortest-path length between waypoints (routes may skip direct edges)
        travel = sum(g.shortest_path_length(route[k], route[k+1]) for k in range(len(route)-1))
        search = sum(g.search_time(v) for v in route if v != base)
        battery_usage[d] = travel + search

    return Solution(
        routes=routes,
        schedule={0: sched0, 1: sched1},
        makespan=ms,
        total_flight_time=sum(
            g.shortest_path_length(r[k], r[k+1])
            for r in routes.values() for k in range(len(r)-1)
        ),
        battery_usage=battery_usage,
        solver="manual",
    )


def test_valid_solution_has_no_violations(graph, base_battery):
    sol = _make_valid_solution(graph)
    violations = validate_solution(sol, graph, 2, base_battery)
    assert violations == [], f"Unexpected violations: {violations}"


def test_missing_coverage(graph, base_battery):
    """Removing a node from all routes should trigger a coverage violation."""
    sol = _make_valid_solution(graph)
    # Remove node 5 from drone 1's route and schedule
    sol.routes[1] = [v for v in sol.routes[1] if v != 5]
    sol.schedule[1] = [(v, a, d) for v, a, d in sol.schedule[1] if v != 5]
    violations = validate_solution(sol, graph, 2, base_battery)
    assert any("5" in v and "covered" in v for v in violations), \
        f"Expected coverage violation for node 5. Got: {violations}"


def test_duplicate_coverage(graph, base_battery):
    """A node searched by both drones should be flagged."""
    sol = _make_valid_solution(graph)
    # Add node 2 to drone 1's schedule AND route — node 2 is already in drone 0's schedule
    route1 = list(sol.routes[1])
    route1.insert(-1, 2)
    sol.routes[1] = route1
    # Also add to schedule so coverage check (based on schedule) detects the duplicate
    arrive2 = 99.0
    sol.schedule[1] = sol.schedule[1] + [(2, arrive2, arrive2 + graph.search_time(2))]
    violations = validate_solution(sol, graph, 2, base_battery)
    assert any("2" in v and "multiple" in v for v in violations), \
        f"Expected duplicate coverage violation. Got: {violations}"


def test_battery_exceeded(graph, base_battery):
    """Tiny battery budget should trigger battery violation."""
    sol = _make_valid_solution(graph)
    tiny_budget = [1.0, 1.0]  # 1 minute — way too small
    violations = validate_solution(sol, graph, 2, tiny_budget)
    assert any("battery" in v.lower() for v in violations), \
        f"Expected battery violation. Got: {violations}"


def test_collision_detected(graph, base_battery):
    """Two drones arriving at the same node at the same time should be flagged."""
    sol = _make_valid_solution(graph)
    # Force both drones to "visit" node 1 at time 0-5 (fabricate schedules)
    sol.schedule[0] = [(1, 0.0, 5.0)]
    sol.schedule[1] = [(1, 2.0, 7.0)]  # overlaps [0,5]
    violations = validate_solution(sol, graph, 2, base_battery)
    assert any("ollision" in v or "node 1" in v for v in violations), \
        f"Expected collision violation. Got: {violations}"


def test_route_not_starting_at_base(graph, base_battery):
    sol = _make_valid_solution(graph)
    sol.routes[0] = [99] + sol.routes[0][1:]  # wrong start
    violations = validate_solution(sol, graph, 2, base_battery)
    assert any("start" in v.lower() or "base" in v.lower() for v in violations), \
        f"Expected base-start violation. Got: {violations}"


def test_route_not_ending_at_base(graph, base_battery):
    sol = _make_valid_solution(graph)
    sol.routes[0] = sol.routes[0][:-1] + [99]  # wrong end
    violations = validate_solution(sol, graph, 2, base_battery)
    assert any("end" in v.lower() or "base" in v.lower() for v in violations), \
        f"Expected base-end violation. Got: {violations}"
