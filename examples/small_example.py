"""Small example: 2x3 grid, 2 drones, battery=60min. MILP + ACO comparison."""
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

from graph_generator import generate_grid_graph
from milp_solver import MILPSolver
from aco_solver import ACOSolver
from constraints import validate_solution
from visualization import (
    plot_graph, plot_solution, plot_gantt,
    plot_timeline, plot_convergence, plot_comparison,
)

OUT = os.path.join(os.path.dirname(__file__), "..", "output")


def main() -> None:
    print("=" * 60)
    print("Small example: 2x3 grid, 2 drones, battery=60 min")
    print("=" * 60)

    graph = generate_grid_graph(rows=2, cols=3, seed=42)
    n_drones = 2
    # The 2x3 grid is sparse — MILP's transit=search constraint forces all
    # search nodes into one drone's route (no feasible 2-partition for this
    # topology). Battery=120 min is enough for that single-drone tour.
    battery = [120.0, 120.0]

    print(f"Graph: {graph.n_nodes()} nodes, base={graph.base_node}")
    for v in graph.search_nodes():
        print(f"  node {v}: search_time={graph.search_time(v):.1f} min")

    plot_graph(graph, save_path=f"{OUT}/small_graph.png", show=False)

    # ---- MILP -------------------------------------------------------
    print("\n--- MILP solver ---")
    milp = MILPSolver(graph, n_drones, battery)
    milp_sol = milp.solve(time_limit=120)

    if milp_sol:
        print(milp_sol)
        violations = validate_solution(milp_sol, graph, n_drones, battery)
        print(f"Violations: {violations if violations else 'none'}")
        plot_solution(graph, milp_sol, save_path=f"{OUT}/small_milp_routes.png", show=False,
                      title="MILP Routes")
        plot_gantt(milp_sol, save_path=f"{OUT}/small_milp_gantt.png", show=False,
                   title="MILP Gantt")
        plot_timeline(milp_sol, save_path=f"{OUT}/small_milp_timeline.png", show=False,
                      title="MILP Timeline")
    else:
        print("MILP: no solution found")

    # ---- ACO --------------------------------------------------------
    print("\n--- ACO solver ---")
    aco = ACOSolver(graph, n_drones, battery, n_ants=20, max_iter=100, seed=42)
    aco_sol = aco.solve()

    if aco_sol:
        print(aco_sol)
        violations = validate_solution(aco_sol, graph, n_drones, battery)
        print(f"Violations: {violations if violations else 'none'}")
        plot_solution(graph, aco_sol, save_path=f"{OUT}/small_aco_routes.png", show=False,
                      title="ACO Routes")
        plot_gantt(aco_sol, save_path=f"{OUT}/small_aco_gantt.png", show=False,
                   title="ACO Gantt")
        plot_convergence(aco.convergence_history, save_path=f"{OUT}/small_aco_convergence.png",
                         show=False)
    else:
        print("ACO: no solution found")

    # ---- Comparison ------------------------------------------------
    plot_comparison(milp_sol, aco_sol, save_path=f"{OUT}/small_comparison.png", show=False)

    if milp_sol and aco_sol:
        gap = (aco_sol.makespan - milp_sol.makespan) / milp_sol.makespan * 100
        print(f"\nMILP makespan : {milp_sol.makespan:.2f} min")
        print(f"ACO  makespan : {aco_sol.makespan:.2f} min")
        print(f"Gap           : {gap:.1f}%")

    print(f"\nFigures saved to {OUT}/")


if __name__ == "__main__":
    main()
