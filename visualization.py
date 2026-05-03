"""Visualisation utilities for drone search planning."""
from __future__ import annotations

import logging
import os
from typing import Optional

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx
import numpy as np

from graph import SearchGraph
from solution import Solution

logger = logging.getLogger(__name__)

# Drone colour palette
_DRONE_COLORS = [
    "#e41a1c", "#377eb8", "#4daf4a", "#984ea3",
    "#ff7f00", "#a65628", "#f781bf", "#999999",
]


def _drone_color(d: int) -> str:
    return _DRONE_COLORS[d % len(_DRONE_COLORS)]


def _save_or_show(fig: plt.Figure, path: Optional[str], show: bool) -> None:
    if path:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        logger.info("Saved figure to %s", path)
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# 1. Graph plot
# ---------------------------------------------------------------------------


def plot_graph(
    graph: SearchGraph,
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "Search Graph",
) -> None:
    """Draw the graph with edge/node weights; base node highlighted."""
    pos = graph.get_layout()
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title(title)

    node_colors = [
        "#e74c3c" if v == graph.base_node else "#3498db" for v in graph.nodes()
    ]
    node_labels = {
        v: f"{v}\n({graph.search_time(v):.1f}m)" if v != graph.base_node else f"BASE\n{v}"
        for v in graph.nodes()
    }

    nx_graph = nx.DiGraph()
    for v in graph.nodes():
        nx_graph.add_node(v)
    for i, j, d in graph.edges():
        if i < j:
            nx_graph.add_edge(i, j, travel_time=d["travel_time"])

    nx.draw_networkx_nodes(nx_graph, pos, node_color=node_colors, node_size=800, ax=ax)
    nx.draw_networkx_labels(nx_graph, pos, labels=node_labels, font_size=8, ax=ax)
    nx.draw_networkx_edges(nx_graph, pos, ax=ax, arrows=False, edge_color="#7f8c8d", width=2)

    edge_labels = {
        (i, j): f"{d['travel_time']:.1f}m"
        for i, j, d in nx_graph.edges(data=True)
    }
    nx.draw_networkx_edge_labels(nx_graph, pos, edge_labels=edge_labels, font_size=7, ax=ax)

    base_patch = mpatches.Patch(color="#e74c3c", label="Base node")
    node_patch = mpatches.Patch(color="#3498db", label="Search node (label=search_time)")
    ax.legend(handles=[base_patch, node_patch], loc="upper right")
    ax.axis("off")

    _save_or_show(fig, save_path, show)


# ---------------------------------------------------------------------------
# 2. Solution plot
# ---------------------------------------------------------------------------


def plot_solution(
    graph: SearchGraph,
    solution: Solution,
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "Mission Routes",
) -> None:
    """Draw drone routes on the graph with directional arrows and visit order."""
    pos = graph.get_layout()
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.set_title(f"{title}  (makespan={solution.makespan:.1f} min)")

    # Background graph
    nx_graph = nx.Graph()
    for v in graph.nodes():
        nx_graph.add_node(v)
    nx.draw_networkx_nodes(
        nx_graph, pos,
        node_color=["#e74c3c" if v == graph.base_node else "#ecf0f1" for v in graph.nodes()],
        node_size=600, ax=ax, edgecolors="#7f8c8d", linewidths=1,
    )
    base_label = {graph.base_node: "BASE"}
    search_labels = {v: str(v) for v in graph.nodes() if v != graph.base_node}
    nx.draw_networkx_labels(nx_graph, pos, labels={**base_label, **search_labels}, font_size=9, ax=ax)

    # Draw routes per drone
    legend_handles = []
    for d, route in solution.routes.items():
        color = _drone_color(d)
        for k in range(len(route) - 1):
            i, j = route[k], route[k + 1]
            xi, yi = pos[i]
            xj, yj = pos[j]
            ax.annotate(
                "",
                xy=(xj, yj), xytext=(xi, yi),
                arrowprops=dict(
                    arrowstyle="->",
                    color=color,
                    lw=2,
                    connectionstyle=f"arc3,rad={0.1 * d}",
                ),
            )

        # Visit order labels next to nodes
        visit_order = [v for v in route if v != graph.base_node]
        for order, v in enumerate(visit_order, start=1):
            px, py = pos[v]
            ax.text(px + 0.03, py + 0.03, str(order), fontsize=7, color=color, fontweight="bold")

        legend_handles.append(mpatches.Patch(color=color, label=f"Drone {d}"))

    ax.legend(handles=legend_handles, loc="upper right")
    ax.axis("off")
    _save_or_show(fig, save_path, show)


# ---------------------------------------------------------------------------
# 3. Gantt chart
# ---------------------------------------------------------------------------


def plot_gantt(
    solution: Solution,
    graph: Optional[SearchGraph] = None,
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "Mission Gantt Chart",
) -> None:
    """Gantt chart: X = time, Y = drone, blocks = search periods."""
    fig, ax = plt.subplots(figsize=(14, 4 + solution.makespan / 30))
    ax.set_title(title)

    # Collect all visited nodes for a consistent colour map
    all_nodes = sorted({node for sched in solution.schedule.values() for node, _, _ in sched})
    cmap = plt.cm.get_cmap("tab20", max(len(all_nodes), 1))
    node_color_map = {v: cmap(i) for i, v in enumerate(all_nodes)}

    yticks, ylabels = [], []
    for d, sched in sorted(solution.schedule.items()):
        y = d
        yticks.append(y)
        ylabels.append(f"Drone {d}")
        for node, arrive, depart in sched:
            color = node_color_map.get(node, "grey")
            ax.barh(y, depart - arrive, left=arrive, height=0.5, color=color, edgecolor="white")
            ax.text(
                (arrive + depart) / 2, y, str(node),
                ha="center", va="center", fontsize=8, color="white", fontweight="bold",
            )

    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels)
    ax.set_xlabel("Time (min)")
    ax.axvline(solution.makespan, color="black", linestyle="--", lw=1.5, label=f"Makespan={solution.makespan:.1f}")
    ax.legend(loc="upper right")
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    plt.tight_layout()
    _save_or_show(fig, save_path, show)


# ---------------------------------------------------------------------------
# 4. Timeline (anti-collision visualisation)
# ---------------------------------------------------------------------------


def plot_timeline(
    solution: Solution,
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "Node Occupation Timeline",
) -> None:
    """Shows when each node is being searched and by which drone."""
    all_nodes = sorted({node for sched in solution.schedule.values() for node, _, _ in sched})
    if not all_nodes:
        logger.warning("No search nodes in schedule — skipping timeline plot")
        return

    fig, ax = plt.subplots(figsize=(14, max(4, len(all_nodes) * 0.6)))
    ax.set_title(title)

    node_y = {v: i for i, v in enumerate(all_nodes)}

    for d, sched in solution.schedule.items():
        color = _drone_color(d)
        for node, arrive, depart in sched:
            y = node_y[node]
            ax.barh(y, depart - arrive, left=arrive, height=0.4, color=color, alpha=0.8)
            ax.text(
                (arrive + depart) / 2, y, f"D{d}",
                ha="center", va="center", fontsize=7, color="white",
            )

    ax.set_yticks(list(node_y.values()))
    ax.set_yticklabels([f"Node {v}" for v in all_nodes])
    ax.set_xlabel("Time (min)")
    ax.axvline(solution.makespan, color="black", linestyle="--", lw=1.5,
                label=f"Makespan={solution.makespan:.1f}")

    legend_handles = [
        mpatches.Patch(color=_drone_color(d), label=f"Drone {d}")
        for d in sorted(solution.schedule.keys())
    ]
    ax.legend(handles=legend_handles, loc="upper right")
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    plt.tight_layout()
    _save_or_show(fig, save_path, show)


# ---------------------------------------------------------------------------
# 5. Convergence curve
# ---------------------------------------------------------------------------


def plot_convergence(
    history: list[float],
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "ACO Convergence",
) -> None:
    """Plot best makespan per ACO iteration."""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title(title)
    ax.plot(history, color="#2980b9", lw=2)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Best makespan (min)")
    ax.grid(linestyle="--", alpha=0.5)
    plt.tight_layout()
    _save_or_show(fig, save_path, show)


# ---------------------------------------------------------------------------
# 6. Solver comparison
# ---------------------------------------------------------------------------


def plot_comparison(
    milp_solution: Optional[Solution],
    aco_solution: Optional[Solution],
    save_path: Optional[str] = None,
    show: bool = True,
    title: str = "MILP vs ACO Comparison",
) -> None:
    """Bar chart comparing makespan, solve time, and total battery usage."""
    labels = []
    makespans = []
    solve_times = []
    battery_totals = []

    for name, sol in [("MILP", milp_solution), ("ACO", aco_solution)]:
        if sol is None:
            continue
        labels.append(name)
        makespans.append(sol.makespan)
        solve_times.append(sol.solve_time)
        battery_totals.append(sum(sol.battery_usage.values()))

    if not labels:
        logger.warning("No solutions to compare")
        return

    x = np.arange(len(labels))
    width = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle(title)

    for ax, values, ylabel, color in zip(
        axes,
        [makespans, solve_times, battery_totals],
        ["Makespan (min)", "Solve time (s)", "Total battery (min)"],
        ["#3498db", "#e74c3c", "#2ecc71"],
    ):
        bars = ax.bar(x, values, color=color, width=0.5, edgecolor="white")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_ylabel(ylabel)
        ax.bar_label(bars, fmt="%.1f", padding=3)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

    plt.tight_layout()
    _save_or_show(fig, save_path, show)
