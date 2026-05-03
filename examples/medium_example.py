"""Medium example: 20 nodes random graph, 4 drones, battery=90 min. ACO only."""
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

from graph_generator import generate_random_graph
from aco_solver import ACOSolver
from constraints import validate_solution
from visualization import plot_graph, plot_solution, plot_gantt, plot_convergence

OUT = os.path.join(os.path.dirname(__file__), "..", "output")


def main() -> None:
    print("=" * 60)
    print("Medium example: 20 nodes, 4 drones, battery=90 min  (ACO)")
    print("=" * 60)

    graph = generate_random_graph(n=20, density=0.4, seed=7)
    n_drones = 4
    battery = [90.0] * n_drones

    print(f"Graph: {graph.n_nodes()} nodes, base={graph.base_node}")

    plot_graph(graph, save_path=f"{OUT}/medium_graph.png", show=False, title="Medium Graph (20 nodes)")

    aco = ACOSolver(
        graph, n_drones, battery,
        alpha=2.0, beta=3.0, rho=0.3,
        n_ants=40, max_iter=300,
        seed=42,
    )
    sol = aco.solve()

    if sol:
        print(sol)
        violations = validate_solution(sol, graph, n_drones, battery)
        print(f"Violations: {violations if violations else 'none'}")
        plot_solution(graph, sol, save_path=f"{OUT}/medium_aco_routes.png", show=False,
                      title="ACO Routes (20 nodes)")
        plot_gantt(sol, save_path=f"{OUT}/medium_aco_gantt.png", show=False, title="ACO Gantt (20 nodes)")
        plot_convergence(aco.convergence_history, save_path=f"{OUT}/medium_aco_convergence.png",
                         show=False, title="ACO Convergence (20 nodes)")
    else:
        print("ACO: no solution found")

    print(f"\nFigures saved to {OUT}/")


if __name__ == "__main__":
    main()
