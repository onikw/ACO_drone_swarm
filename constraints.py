"""Validation of drone mission solutions against all problem constraints."""
from __future__ import annotations

import logging

from graph import SearchGraph
from solution import Solution

logger = logging.getLogger(__name__)

_EPS = 1e-6  # numerical tolerance


def validate_solution(
    solution: Solution,
    graph: SearchGraph,
    n_drones: int,
    battery_budgets: list[float],
) -> list[str]:
    """Check all constraints and return a list of violation messages.

    An empty list means the solution is feasible.
    """
    violations: list[str] = []
    base = graph.base_node
    search_nodes = set(graph.search_nodes())

    # ------------------------------------------------------------------
    # 1. Coverage: every search node is searched (scheduled) exactly once.
    # Routes may contain transit nodes — coverage is determined by the
    # schedule (which only lists searched nodes), not the route.
    # ------------------------------------------------------------------
    visited: dict[int, list[int]] = {v: [] for v in search_nodes}
    for d, sched in solution.schedule.items():
        for node, _, _ in sched:
            if node in visited:
                visited[node].append(d)

    for v, drones in visited.items():
        if len(drones) == 0:
            violations.append(f"Node {v} not covered by any drone")
        elif len(drones) > 1:
            violations.append(f"Node {v} covered by multiple drones: {drones}")

    # ------------------------------------------------------------------
    # 2. Routes start and end at base
    # ------------------------------------------------------------------
    for d, route in solution.routes.items():
        if not route:
            violations.append(f"Drone {d} has an empty route")
            continue
        if route[0] != base:
            violations.append(f"Drone {d} route does not start at base {base}")
        if route[-1] != base:
            violations.append(f"Drone {d} route does not end at base {base}")

    # ------------------------------------------------------------------
    # 3. Path validity (every consecutive pair must be reachable)
    # Routes may contain logical waypoints; direct edges are not required.
    # ------------------------------------------------------------------
    for d, route in solution.routes.items():
        for k in range(len(route) - 1):
            i, j = route[k], route[k + 1]
            if graph.shortest_path_length(i, j) == float("inf"):
                violations.append(f"Drone {d}: no path from {i} to {j} in graph")

    # ------------------------------------------------------------------
    # 4. Battery budget
    # ------------------------------------------------------------------
    for d, route in solution.routes.items():
        budget = battery_budgets[d] if d < len(battery_budgets) else float("inf")
        total = 0.0
        for k in range(len(route) - 1):
            i, j = route[k], route[k + 1]
            total += graph.shortest_path_length(i, j)
        # Search time: only for nodes that appear in the schedule (searched nodes).
        # Transit nodes are in the route but are NOT searched.
        searched_by_d = {node for node, _, _ in solution.schedule.get(d, [])}
        for v in searched_by_d:
            total += graph.search_time(v)
        usage = solution.battery_usage.get(d, total)
        if usage > budget + _EPS:
            violations.append(
                f"Drone {d} battery usage {usage:.2f} exceeds budget {budget:.2f}"
            )

    # ------------------------------------------------------------------
    # 5. Schedule consistency (timing matches route)
    # ------------------------------------------------------------------
    for d, sched in solution.schedule.items():
        route = solution.routes.get(d, [])
        route_set = set(route)
        sched_nodes = [v for v, _, _ in sched if v != base]

        # Every scheduled node must appear in the route.
        for v in sched_nodes:
            if v not in route_set:
                violations.append(
                    f"Drone {d}: scheduled node {v} not in route {route}"
                )

        # Scheduled nodes must appear in route order (as a subsequence).
        # Routes may additionally contain transit nodes not in the schedule.
        route_no_base = [v for v in route if v != base]
        idx = 0
        for v in sched_nodes:
            while idx < len(route_no_base) and route_no_base[idx] != v:
                idx += 1
            if idx >= len(route_no_base):
                violations.append(
                    f"Drone {d}: schedule node {v} out of order in route {route}"
                )
                break
            idx += 1

        # Check timing feasibility
        prev_node = base
        prev_depart = 0.0
        for node, arrive, depart in sched:
            if node == base:
                continue
            min_arrive = prev_depart + graph.shortest_path_length(prev_node, node)  # Dijkstra path
            if arrive < min_arrive - _EPS:
                violations.append(
                    f"Drone {d}: arrives at {node} at {arrive:.2f} but earliest is {min_arrive:.2f}"
                )
            expected_depart = arrive + graph.search_time(node)
            if abs(depart - expected_depart) > _EPS:
                violations.append(
                    f"Drone {d}: depart {depart:.2f} at node {node} doesn't match "
                    f"arrive {arrive:.2f} + search_time {graph.search_time(node):.2f}"
                )
            prev_node = node
            prev_depart = depart

    # ------------------------------------------------------------------
    # 6. Anti-collision (no two drones at the same node at the same time)
    # ------------------------------------------------------------------
    # Build per-node list of (t_start, t_end, drone_id)
    node_windows: dict[int, list[tuple[float, float, int]]] = {}
    for d, sched in solution.schedule.items():
        for node, arrive, depart in sched:
            if node == base:
                continue
            node_windows.setdefault(node, []).append((arrive, depart, d))

    for node, windows in node_windows.items():
        windows.sort()
        for k in range(len(windows) - 1):
            t_start_a, t_end_a, d_a = windows[k]
            t_start_b, t_end_b, d_b = windows[k + 1]
            # Overlap if one starts before the other ends
            if t_start_b < t_end_a - _EPS:
                violations.append(
                    f"Collision at node {node}: drone {d_a} [{t_start_a:.2f}, {t_end_a:.2f}] "
                    f"overlaps drone {d_b} [{t_start_b:.2f}, {t_end_b:.2f}]"
                )

    # ------------------------------------------------------------------
    # 7. Makespan matches schedule
    # ------------------------------------------------------------------
    computed_makespan = 0.0
    for d, sched in solution.schedule.items():
        route = solution.routes.get(d, [])
        if not sched:
            continue
        last_node, last_arrive, last_depart = sched[-1]
        # Return to base after last node
        if last_node != base:
            return_time = last_depart + graph.shortest_path_length(last_node, base)
        else:
            return_time = last_depart
        computed_makespan = max(computed_makespan, return_time)

    if abs(solution.makespan - computed_makespan) > _EPS + 0.01:
        violations.append(
            f"Makespan mismatch: solution reports {solution.makespan:.2f} "
            f"but computed {computed_makespan:.2f}"
        )

    if violations:
        logger.warning("Solution has %d violation(s)", len(violations))
    else:
        logger.info("Solution passed all constraint checks")

    return violations
