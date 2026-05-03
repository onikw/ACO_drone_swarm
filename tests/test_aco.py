"""ACO solver tests — feasibility and comparison with MILP."""
import pytest

from aco_solver import ACOSolver, TimeReservationTable
from milp_solver import MILPSolver
from constraints import validate_solution


def test_aco_finds_solution(small_graph, n_drones, small_battery):
    solver = ACOSolver(small_graph, n_drones, small_battery, n_ants=20, max_iter=50, seed=0)
    sol = solver.solve()
    assert sol is not None, "ACO should find a solution on small graph"


def test_aco_solution_feasible(small_graph, n_drones, small_battery):
    solver = ACOSolver(small_graph, n_drones, small_battery, n_ants=20, max_iter=50, seed=0)
    sol = solver.solve()
    assert sol is not None
    violations = validate_solution(sol, small_graph, n_drones, small_battery)
    assert violations == [], f"ACO solution has violations: {violations}"


def test_aco_coverage(small_graph, n_drones, small_battery):
    solver = ACOSolver(small_graph, n_drones, small_battery, n_ants=20, max_iter=50, seed=0)
    sol = solver.solve()
    assert sol is not None
    search_nodes = set(small_graph.search_nodes())
    covered = set()
    for route in sol.routes.values():
        for v in route:
            if v in search_nodes:
                covered.add(v)
    assert covered == search_nodes, f"ACO did not cover all nodes. Missing: {search_nodes - covered}"


def test_aco_convergence_history(small_graph, n_drones, small_battery):
    n_iter = 30
    solver = ACOSolver(small_graph, n_drones, small_battery, n_ants=10, max_iter=n_iter, seed=1)
    solver.solve()
    assert len(solver.convergence_history) == n_iter
    # Convergence history should be non-increasing (best found so far)
    for i in range(1, len(solver.convergence_history)):
        assert solver.convergence_history[i] <= solver.convergence_history[i - 1] + 1e-6


def test_aco_vs_milp_makespan(small_graph, n_drones, small_battery):
    """ACO should produce a solution within 3× the MILP optimal (loose sanity check)."""
    milp_sol = MILPSolver(small_graph, n_drones, small_battery).solve(time_limit=120)
    aco_sol = ACOSolver(small_graph, n_drones, small_battery, n_ants=30, max_iter=150, seed=0).solve()
    assert milp_sol is not None and aco_sol is not None
    assert aco_sol.makespan <= milp_sol.makespan * 3.0, (
        f"ACO makespan {aco_sol.makespan:.1f} is more than 3× MILP {milp_sol.makespan:.1f}"
    )


# ---------------------------------------------------------------------------
# TimeReservationTable unit tests
# ---------------------------------------------------------------------------


def test_trt_no_conflict_empty():
    trt = TimeReservationTable()
    assert not trt.check_conflict(1, 0, 10)


def test_trt_reserves_and_detects_conflict():
    trt = TimeReservationTable()
    trt.reserve(1, 5, 15)
    assert trt.check_conflict(1, 10, 20)   # overlaps
    assert not trt.check_conflict(1, 15, 25)  # adjacent, no overlap


def test_trt_earliest_free():
    trt = TimeReservationTable()
    trt.reserve(1, 5, 15)
    # [0, 3) does NOT overlap [5, 15) → earliest free start is 0
    assert trt.earliest_free(1, 3, 0) == pytest.approx(0.0)
    # [0, 12) DOES overlap [5, 15) → pushed past 15
    assert trt.earliest_free(1, 12, 0) == pytest.approx(15.0)
    # No reservation on node 2 → earliest is not_before
    assert trt.earliest_free(2, 3, 10) == pytest.approx(10.0)
