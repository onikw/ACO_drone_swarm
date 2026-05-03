"""MILP solver tests on a small 2x3 grid."""
import pytest

from milp_solver import MILPSolver
from constraints import validate_solution


def test_milp_finds_solution(small_graph, n_drones, small_battery):
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None, "MILP should find a solution on small graph"


def test_milp_solution_feasible(small_graph, n_drones, small_battery):
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None
    violations = validate_solution(sol, small_graph, n_drones, small_battery)
    assert violations == [], f"MILP solution has violations: {violations}"


def test_milp_coverage(small_graph, n_drones, small_battery):
    """Every search node must appear in exactly one drone's route."""
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None

    search_nodes = set(small_graph.search_nodes())
    visited: dict[int, list[int]] = {v: [] for v in search_nodes}
    for d, route in sol.routes.items():
        for v in route:
            if v in visited:
                visited[v].append(d)

    for v, drones in visited.items():
        assert len(drones) == 1, f"Node {v} covered {len(drones)} times (expected 1)"


def test_milp_routes_start_end_at_base(small_graph, n_drones, small_battery):
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None
    base = small_graph.base_node
    for d, route in sol.routes.items():
        assert route[0] == base, f"Drone {d} route doesn't start at base"
        assert route[-1] == base, f"Drone {d} route doesn't end at base"


def test_milp_battery_respected(small_graph, n_drones, small_battery):
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None
    for d, usage in sol.battery_usage.items():
        budget = small_battery[d]
        assert usage <= budget + 1e-4, f"Drone {d} used {usage:.2f} > budget {budget}"


def test_milp_makespan_positive(small_graph, n_drones, small_battery):
    solver = MILPSolver(small_graph, n_drones, small_battery)
    sol = solver.solve(time_limit=120)
    assert sol is not None
    assert sol.makespan > 0
