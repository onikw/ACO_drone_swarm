"""Solver comparison across increasing graph sizes (6–20 nodes)."""
import logging
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

logging.basicConfig(level=logging.WARNING)

from graph_generator import generate_random_graph
from milp_solver import MILPSolver
from aco_solver import ACOSolver
from constraints import validate_solution

OUT = os.path.join(os.path.dirname(__file__), "..", "output")


def run_instance(n_nodes: int, n_drones: int, battery: float, seed: int) -> dict:
    graph = generate_random_graph(n=n_nodes, density=0.5, seed=seed)
    budgets = [battery] * n_drones

    # MILP
    milp_makespan = None
    milp_time = None
    milp_feasible = False
    try:
        t0 = time.time()
        milp_sol = MILPSolver(graph, n_drones, budgets).solve(time_limit=60)
        milp_time = time.time() - t0
        if milp_sol:
            milp_makespan = milp_sol.makespan
            milp_feasible = True
    except Exception as exc:
        milp_time = time.time() - t0
        logging.warning("MILP failed for n=%d: %s", n_nodes, exc)

    # ACO
    aco = ACOSolver(graph, n_drones, budgets, n_ants=30, max_iter=200, seed=seed)
    t0 = time.time()
    aco_sol = aco.solve()
    aco_time = time.time() - t0
    aco_makespan = aco_sol.makespan if aco_sol else None

    gap = None
    if milp_makespan and aco_makespan:
        gap = (aco_makespan - milp_makespan) / milp_makespan * 100

    return {
        "n_nodes": n_nodes,
        "n_drones": n_drones,
        "milp_makespan": milp_makespan,
        "milp_time": milp_time,
        "aco_makespan": aco_makespan,
        "aco_time": aco_time,
        "gap_pct": gap,
    }


def main() -> None:
    sizes = [6, 8, 10, 12, 15, 20]
    results = []

    header = f"{'n':>4} {'#D':>3} {'MILP ms':>10} {'MILP t':>8} {'ACO ms':>10} {'ACO t':>8} {'gap%':>8}"
    print(header)
    print("-" * len(header))

    for n in sizes:
        n_drones = max(2, n // 5)
        battery = 90.0
        r = run_instance(n, n_drones, battery, seed=42)
        results.append(r)

        milp_str = f"{r['milp_makespan']:>10.2f}" if r["milp_makespan"] else "      None"
        aco_str = f"{r['aco_makespan']:>10.2f}" if r["aco_makespan"] else "      None"
        gap_str = f"{r['gap_pct']:>8.1f}" if r["gap_pct"] is not None else "       N/A"
        print(
            f"{r['n_nodes']:>4} {r['n_drones']:>3} {milp_str} {r['milp_time']:>8.1f}s "
            f"{aco_str} {r['aco_time']:>8.1f}s {gap_str}%"
        )

    # Save comparison chart
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle("MILP vs ACO: scaling comparison")

        ns = [r["n_nodes"] for r in results]
        milp_ms = [r["milp_makespan"] or 0 for r in results]
        aco_ms = [r["aco_makespan"] or 0 for r in results]
        milp_t = [r["milp_time"] or 0 for r in results]
        aco_t = [r["aco_time"] or 0 for r in results]

        x = np.arange(len(ns))
        w = 0.35
        axes[0].bar(x - w / 2, milp_ms, w, label="MILP", color="#3498db")
        axes[0].bar(x + w / 2, aco_ms, w, label="ACO", color="#e74c3c")
        axes[0].set_xticks(x)
        axes[0].set_xticklabels([str(n) for n in ns])
        axes[0].set_xlabel("Graph size (nodes)")
        axes[0].set_ylabel("Makespan (min)")
        axes[0].set_title("Solution quality")
        axes[0].legend()

        axes[1].plot(ns, milp_t, "o-", label="MILP", color="#3498db")
        axes[1].plot(ns, aco_t, "s-", label="ACO", color="#e74c3c")
        axes[1].set_xlabel("Graph size (nodes)")
        axes[1].set_ylabel("Solve time (s)")
        axes[1].set_title("Computation time")
        axes[1].legend()

        plt.tight_layout()
        path = f"{OUT}/comparison_scaling.png"
        os.makedirs(OUT, exist_ok=True)
        plt.savefig(path, dpi=150, bbox_inches="tight")
        print(f"\nScaling chart saved to {path}")
        plt.close()
    except Exception as exc:
        print(f"Chart generation failed: {exc}")


if __name__ == "__main__":
    main()
