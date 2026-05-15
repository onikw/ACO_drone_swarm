"""CLI entry point for the drone swarm ACO solver.

Usage examples
--------------
  python main.py                                   # default config.json
  python main.py --config my.json                  # custom config file
  python main.py --solver aco                      # override solver
  python main.py --graph-type random               # override graph type
  python main.py --n-drones 3 --battery 90 90 90  # override mission params
  python main.py --aco-n-ants 50 --aco-max-iter 300
  python main.py --milp-time-limit 60
  python main.py --no-save --show                  # display plots interactively
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from typing import Optional


# ---------------------------------------------------------------------------
# Config loading and merging
# ---------------------------------------------------------------------------


def _load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _apply_overrides(cfg: dict, args: argparse.Namespace) -> dict:
    """Merge CLI overrides into the loaded config dict (CLI wins)."""

    if args.solver is not None:
        cfg["solver"] = args.solver

    if args.graph_file is not None:
        cfg["graph"]["type"] = "file"
        cfg["graph"].setdefault("file", {})["path"] = args.graph_file
    elif args.graph_type is not None:
        cfg["graph"]["type"] = args.graph_type

    # --- mission ---
    m = cfg["mission"]
    if args.n_drones is not None:
        m["n_drones"] = args.n_drones
    if args.battery is not None:
        n = m["n_drones"]
        if len(args.battery) == 1:
            m["battery_budgets"] = [args.battery[0]] * n
        else:
            m["battery_budgets"] = args.battery

    # --- graph sub-sections ---
    gtype = cfg["graph"]["type"]
    gsub = cfg["graph"].get(gtype, {})
    if args.graph_seed is not None:
        gsub["seed"] = args.graph_seed
    if args.graph_base_node is not None:
        gsub["base_node"] = args.graph_base_node
    # grid
    if args.grid_rows is not None:
        gsub["rows"] = args.grid_rows
    if args.grid_cols is not None:
        gsub["cols"] = args.grid_cols
    # random
    if args.random_n is not None:
        gsub["n"] = args.random_n
    if args.random_density is not None:
        gsub["density"] = args.random_density
    # cluster
    if args.cluster_n_clusters is not None:
        gsub["n_clusters"] = args.cluster_n_clusters
    if args.cluster_nodes_per_cluster is not None:
        gsub["nodes_per_cluster"] = args.cluster_nodes_per_cluster
    cfg["graph"][gtype] = gsub

    # --- ACO ---
    a = cfg["aco"]
    if args.aco_alpha is not None:
        a["alpha"] = args.aco_alpha
    if args.aco_beta is not None:
        a["beta"] = args.aco_beta
    if args.aco_rho is not None:
        a["rho"] = args.aco_rho
    if args.aco_n_ants is not None:
        a["n_ants"] = args.aco_n_ants
    if args.aco_max_iter is not None:
        a["max_iter"] = args.aco_max_iter
    if args.aco_Q is not None:
        a["Q"] = args.aco_Q
    if args.aco_no_local_search:
        a["local_search"] = False
    if args.aco_seed is not None:
        a["seed"] = args.aco_seed
    if args.aco_progress_every is not None:
        a["progress_every"] = args.aco_progress_every

    # --- MILP ---
    mi = cfg["milp"]
    if args.milp_time_limit is not None:
        mi["time_limit"] = args.milp_time_limit
    if args.milp_no_transit:
        mi["transit_allowed"] = False

    # --- output ---
    o = cfg["output"]
    if args.output_dir is not None:
        o["directory"] = args.output_dir
    if args.no_save:
        o["save_plots"] = False
    if args.show:
        o["show_plots"] = True
    if args.log_level is not None:
        o["log_level"] = args.log_level

    return cfg


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="main.py",
        description="Drone swarm area search — MILP / ACO solver",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument(
        "--config", "-c",
        default="config.json",
        metavar="FILE",
        help="JSON config file to load",
    )
    p.add_argument(
        "--solver",
        choices=["milp", "aco", "both"],
        default=None,
        help="Which solver(s) to run",
    )

    # -- graph --
    g = p.add_argument_group("graph")
    g.add_argument("--graph-type", choices=["grid", "random", "cluster", "file"], default=None)
    g.add_argument("--graph-file", default=None, metavar="FILE",
                   help="JSON file with a custom graph (sets --graph-type file)")
    g.add_argument("--graph-seed", type=int, default=None, metavar="N")
    g.add_argument("--graph-base-node", type=int, default=None, metavar="N")
    g.add_argument("--grid-rows", type=int, default=None)
    g.add_argument("--grid-cols", type=int, default=None)
    g.add_argument("--random-n", type=int, default=None, metavar="N",
                   help="Number of nodes (random graph)")
    g.add_argument("--random-density", type=float, default=None, metavar="D")
    g.add_argument("--cluster-n-clusters", type=int, default=None)
    g.add_argument("--cluster-nodes-per-cluster", type=int, default=None)

    # -- mission --
    m = p.add_argument_group("mission")
    m.add_argument("--n-drones", type=int, default=None, metavar="N")
    m.add_argument(
        "--battery", type=float, nargs="+", default=None, metavar="B",
        help="Battery budget(s) in minutes. One value → same for all drones.",
    )

    # -- ACO --
    a = p.add_argument_group("ACO hyperparameters")
    a.add_argument("--aco-alpha", type=float, default=None, metavar="α")
    a.add_argument("--aco-beta", type=float, default=None, metavar="β")
    a.add_argument("--aco-rho", type=float, default=None, metavar="ρ")
    a.add_argument("--aco-n-ants", type=int, default=None)
    a.add_argument("--aco-max-iter", type=int, default=None)
    a.add_argument("--aco-Q", type=float, default=None)
    a.add_argument("--aco-no-local-search", action="store_true", default=False)
    a.add_argument("--aco-seed", type=int, default=None)
    a.add_argument("--aco-progress-every", type=int, default=None, metavar="N",
                   help="Print live progress every N iterations (0 = silent)")

    # -- MILP --
    mi = p.add_argument_group("MILP options")
    mi.add_argument("--milp-time-limit", type=int, default=None, metavar="SEC")
    mi.add_argument("--milp-no-transit", action="store_true", default=False,
                    help="Disable transit-allowed mode (strict formulation)")

    # -- output --
    o = p.add_argument_group("output")
    o.add_argument("--output-dir", default=None, metavar="DIR")
    o.add_argument("--no-save", action="store_true", default=False,
                   help="Don't save plot files")
    o.add_argument("--show", action="store_true", default=False,
                   help="Display plots interactively")
    o.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], default=None)

    return p


# ---------------------------------------------------------------------------
# Graph builder from config
# ---------------------------------------------------------------------------


def _build_graph(cfg: dict):
    from graph_generator import generate_grid_graph, generate_random_graph, generate_cluster_graph
    from graph import SearchGraph

    gtype = cfg["graph"]["type"]
    p = cfg["graph"].get(gtype, {})

    if gtype == "file":
        path = p.get("path", "")
        if not os.path.exists(path):
            print(f"[ERROR] Graph file not found: {path}", file=sys.stderr)
            sys.exit(1)
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return SearchGraph.from_dict(data)

    if gtype == "grid":
        return generate_grid_graph(
            rows=p.get("rows", 2),
            cols=p.get("cols", 3),
            search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
            travel_time_range=tuple(p.get("travel_time_range", [1.0, 5.0])),
            base_node=p.get("base_node", 0),
            seed=p.get("seed"),
        )
    elif gtype == "random":
        return generate_random_graph(
            n=p.get("n", 10),
            density=p.get("density", 0.5),
            search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
            travel_time_range=tuple(p.get("travel_time_range", [1.0, 5.0])),
            base_node=p.get("base_node", 0),
            seed=p.get("seed"),
        )
    elif gtype == "cluster":
        return generate_cluster_graph(
            n_clusters=p.get("n_clusters", 3),
            nodes_per_cluster=p.get("nodes_per_cluster", 4),
            search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
            travel_time_range_intra=tuple(p.get("travel_time_range_intra", [1.0, 3.0])),
            travel_time_range_inter=tuple(p.get("travel_time_range_inter", [3.0, 8.0])),
            base_node=p.get("base_node", 0),
            seed=p.get("seed"),
        )
    else:
        raise ValueError(f"Unknown graph type: {gtype!r}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv: Optional[list[str]] = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if not os.path.exists(args.config):
        print(f"[ERROR] Config file not found: {args.config}", file=sys.stderr)
        sys.exit(1)
    cfg = _load_config(args.config)
    cfg = _apply_overrides(cfg, args)

    logging.basicConfig(
        level=getattr(logging, cfg["output"]["log_level"], logging.INFO),
        format="%(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("main")
    logger.info("Effective config:\n%s", json.dumps(cfg, indent=2))

    out_dir = cfg["output"]["directory"]
    save = cfg["output"]["save_plots"]
    show = cfg["output"]["show_plots"]

    def out(name: str) -> Optional[str]:
        if not save:
            return None
        os.makedirs(out_dir, exist_ok=True)
        return os.path.join(out_dir, name)

    graph = _build_graph(cfg)
    n_drones = cfg["mission"]["n_drones"]
    battery = cfg["mission"]["battery_budgets"]

    # Align battery list length with n_drones
    if len(battery) < n_drones:
        battery = battery + [battery[-1]] * (n_drones - len(battery))
    battery = battery[:n_drones]

    print("=" * 60)
    print(f"Graph : {cfg['graph']['type']}  "
          f"({graph.n_nodes()} nodes, base={graph.base_node})")
    print(f"Drones: {n_drones}  battery={battery} min")
    print(f"Solver: {cfg['solver']}")
    print("=" * 60)

    from visualization import plot_graph, plot_solution, plot_gantt, plot_convergence, plot_comparison
    from constraints import validate_solution

    plot_graph(graph, save_path=out("graph.png"), show=show, title="Search Graph")

    milp_sol = None
    aco_sol = None

    # ---- MILP ---------------------------------------------------------------
    if cfg["solver"] in ("milp", "both"):
        from milp_solver import MILPSolver

        print("\n--- MILP solver ---")
        mc = cfg["milp"]
        milp = MILPSolver(graph, n_drones, battery, transit_allowed=mc["transit_allowed"])
        milp_sol = milp.solve(time_limit=mc["time_limit"])

        if milp_sol:
            print(milp_sol)
            violations = validate_solution(milp_sol, graph, n_drones, battery)
            print(f"Violations : {violations if violations else 'none'}")
            plot_solution(graph, milp_sol, save_path=out("milp_routes.png"), show=show,
                          title="MILP Routes")
            plot_gantt(milp_sol, save_path=out("milp_gantt.png"), show=show, title="MILP Gantt")
        else:
            print("MILP: no feasible solution found")

    # ---- ACO ----------------------------------------------------------------
    if cfg["solver"] in ("aco", "both"):
        from aco_solver import ACOSolver

        print("\n--- ACO solver ---")
        ac = cfg["aco"]
        progress_every = ac.get("progress_every", 10)

        aco = ACOSolver(
            graph, n_drones, battery,
            alpha=ac["alpha"],
            beta=ac["beta"],
            rho=ac["rho"],
            n_ants=ac["n_ants"],
            max_iter=ac["max_iter"],
            Q=ac["Q"],
            local_search=ac["local_search"],
            seed=ac.get("seed"),
        )

        # Build progress callback that reads aco.initial_solution lazily:
        # initial_solution is set inside solve() before the first iteration,
        # so by the time the callback fires it is already populated.
        def _progress_cb(iteration: int, total: int, global_best: float,
                         iter_best: float, elapsed: float) -> None:
            bar_width = 28
            filled = int(bar_width * iteration / total)
            bar = "█" * filled + "░" * (bar_width - filled)
            nn = aco.initial_solution.makespan if aco.initial_solution else None
            vs_nn = f"  vs NN: {(nn - global_best) / nn * 100:+.1f}%" if nn else ""
            print(
                f"\r  [{bar}] {iteration:>4}/{total}"
                f"  best={global_best:.2f}"
                f"  iter={iter_best:.2f}"
                f"  {elapsed:.0f}s"
                f"{vs_nn}",
                end="",
                flush=True,
            )
            if iteration == total:
                print()

        aco_sol = aco.solve(
            progress_callback=_progress_cb if progress_every > 0 else None,
            progress_every=progress_every,
        )

        if progress_every > 0 and ac["max_iter"] % progress_every != 0:
            print()  # ensure newline if last iter wasn't a progress tick

        if aco.initial_solution:
            init = aco.initial_solution
            print(f"\n  Initial (NN) : makespan={init.makespan:.2f} min")
            for d, route in init.routes.items():
                print(f"    Drone {d}: {' → '.join(str(v) for v in route)}")
            plot_solution(graph, init, save_path=out("nn_initial_routes.png"), show=show,
                          title="NN Initial Solution")
        else:
            print("\n  Initial (NN) : no feasible solution found")

        if aco_sol:
            print(f"\n  Final  (ACO) : {aco_sol}")
            violations = validate_solution(aco_sol, graph, n_drones, battery)
            print(f"  Violations   : {violations if violations else 'none'}")
            if aco.initial_solution:
                improvement = (aco.initial_solution.makespan - aco_sol.makespan) / aco.initial_solution.makespan * 100
                print(f"  Improvement over NN : {improvement:+.1f}%")
            print(f"  Best at iteration   : {aco.best_iter}/{ac['max_iter']}")
            plot_solution(graph, aco_sol, save_path=out("aco_routes.png"), show=show,
                          title="ACO Routes")
            plot_gantt(aco_sol, save_path=out("aco_gantt.png"), show=show, title="ACO Gantt")
            plot_convergence(aco.convergence_history, save_path=out("aco_convergence.png"),
                             show=show)
        else:
            print("ACO: no feasible solution found")

    # ---- Comparison ---------------------------------------------------------
    if milp_sol and aco_sol:
        plot_comparison(milp_sol, aco_sol, save_path=out("comparison.png"), show=show)
        gap = (aco_sol.makespan - milp_sol.makespan) / milp_sol.makespan * 100
        print(f"\nMILP makespan : {milp_sol.makespan:.2f} min")
        print(f"ACO  makespan : {aco_sol.makespan:.2f} min")
        print(f"Quality gap   : {gap:+.1f}%")

    if save:
        print(f"\nPlots saved to: {out_dir}/")


if __name__ == "__main__":
    main()