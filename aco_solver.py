"""Ant Colony Optimization solver for multi-drone area search."""
from __future__ import annotations

import copy
import logging
import math
import random
import time
from typing import Optional

from graph import SearchGraph
from solution import Solution

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Time Reservation Table
# ---------------------------------------------------------------------------


class TimeReservationTable:
    """Per-node reservation windows used to enforce anti-collision constraints.

    Each reservation is a half-open interval [t_start, t_end).
    """

    def __init__(self) -> None:
        # node -> sorted list of (t_start, t_end)
        self._windows: dict[int, list[tuple[float, float]]] = {}

    def check_conflict(self, node: int, t_start: float, t_end: float) -> bool:
        """Return True if the interval [t_start, t_end) conflicts with any reserved window."""
        for ws, we in self._windows.get(node, []):
            if t_start < we and t_end > ws:
                return True
        return False

    def reserve(self, node: int, t_start: float, t_end: float) -> None:
        """Add reservation [t_start, t_end) for the node."""
        self._windows.setdefault(node, []).append((t_start, t_end))
        self._windows[node].sort()

    def earliest_free(self, node: int, duration: float, not_before: float = 0.0) -> float:
        """Return the earliest start time ≥ not_before at which duration fits without conflict."""
        t = not_before
        for ws, we in sorted(self._windows.get(node, [])):
            if ws >= t + duration:
                break
            if we > t:
                t = we  # push past this reservation
        return t

    def copy(self) -> "TimeReservationTable":
        trt = TimeReservationTable()
        trt._windows = {k: list(v) for k, v in self._windows.items()}
        return trt


# ---------------------------------------------------------------------------
# Nearest-neighbour heuristic for τ₀ initialisation
# ---------------------------------------------------------------------------


def _nn_tour_length(graph: SearchGraph) -> float:
    """Rough nearest-neighbour tour length for τ₀ calibration."""
    nodes = graph.search_nodes()
    if not nodes:
        return 1.0
    visited = set()
    current = graph.base_node
    total = 0.0
    while len(visited) < len(nodes):
        best_cost = float("inf")
        best_next = None
        for v in nodes:
            if v not in visited and graph.has_edge(current, v):
                c = graph.travel_time(current, v) + graph.search_time(v)
                if c < best_cost:
                    best_cost = c
                    best_next = v
        if best_next is None:
            break
        visited.add(best_next)
        total += best_cost
        current = best_next
    total += graph.shortest_path_length(current, graph.base_node)
    return max(total, 1.0)


# ---------------------------------------------------------------------------
# 2-opt local search within a single drone route
# ---------------------------------------------------------------------------


def _two_opt(route: list[int], graph: SearchGraph) -> list[int]:
    """Improve a single drone route with 2-opt swaps (minimise travel time).

    Uses shortest-path length as edge cost so the route need not use only direct edges.
    Only the search-node subsequence (route[1:-1]) is reordered; base endpoints are fixed.
    """
    if len(route) <= 3:
        return route

    def seg_cost(a: int, b: int) -> float:
        return graph.shortest_path_length(a, b)

    improved = True
    best = route[:]
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                # Cost delta for reversing segment [i..j]
                old = seg_cost(best[i - 1], best[i]) + seg_cost(best[j], best[j + 1])
                new = seg_cost(best[i - 1], best[j]) + seg_cost(best[i], best[j + 1])
                if new < old - 1e-6:
                    best[i:j + 1] = best[i:j + 1][::-1]
                    improved = True
    return best


# ---------------------------------------------------------------------------
# Inter-route relocate: move nodes between drones to reduce makespan
# ---------------------------------------------------------------------------


def _route_cost(route: list[int], graph: SearchGraph, v0: int) -> float:
    """Compute total time for a drone route (travel + search), excluding base search."""
    t = 0.0
    for k in range(len(route) - 1):
        t += graph.shortest_path_length(route[k], route[k + 1])
        if route[k + 1] != v0:
            t += graph.search_time(route[k + 1])
    return t


def _route_battery(route: list[int], graph: SearchGraph, v0: int) -> float:
    """Compute battery usage for a drone route (travel + search)."""
    usage = 0.0
    for k in range(len(route) - 1):
        usage += graph.shortest_path_length(route[k], route[k + 1])
        if route[k + 1] != v0:
            usage += graph.search_time(route[k + 1])
    return usage


def _inter_route_relocate(
    drone_routes: list[list[int]],
    graph: SearchGraph,
    battery_budgets: list[float],
    return_cost: dict[int, float],
    v0: int,
    n_drones: int,
) -> tuple[list[list[int]], list[float]]:
    """Try moving nodes from the busiest drone to less busy ones.

    Greedy first-improvement: find the drone with the highest route cost
    (the bottleneck determining makespan), then try removing each of its
    search nodes and inserting it into the best position of another drone's
    route, accepting the first move that reduces the overall makespan.

    Repeats until no improving move is found.
    """
    routes = [r[:] for r in drone_routes]  # deep copy

    improved = True
    while improved:
        improved = False

        # Compute costs per drone
        costs = [_route_cost(routes[d], graph, v0) for d in range(n_drones)]
        makespan = max(costs)

        # Bottleneck drone
        d_max = int(max(range(n_drones), key=lambda d: costs[d]))

        # Try removing each search node from the bottleneck drone
        route_max = routes[d_max]
        # search nodes are route_max[1:-1] (exclude base at start/end)
        for idx_remove in range(1, len(route_max) - 1):
            node = route_max[idx_remove]

            # Tentatively remove node from d_max
            new_route_max = route_max[:idx_remove] + route_max[idx_remove + 1:]
            new_cost_max = _route_cost(new_route_max, graph, v0)

            # Try inserting node into each other drone
            best_target_d = -1
            best_insert_idx = -1
            best_new_makespan = makespan

            for d_target in range(n_drones):
                if d_target == d_max:
                    continue

                route_target = routes[d_target]
                # Try each insertion position (between route_target[i] and route_target[i+1])
                for idx_insert in range(1, len(route_target)):
                    new_route_target = (
                        route_target[:idx_insert] + [node] + route_target[idx_insert:]
                    )

                    # Battery feasibility check for target drone
                    target_battery = _route_battery(new_route_target, graph, v0)
                    if target_battery > battery_budgets[d_target] + 1e-6:
                        continue

                    new_cost_target = _route_cost(new_route_target, graph, v0)

                    # New makespan: max of all drone costs with this move
                    candidate_makespan = new_cost_max
                    for d_other in range(n_drones):
                        if d_other == d_max:
                            continue
                        elif d_other == d_target:
                            candidate_makespan = max(candidate_makespan, new_cost_target)
                        else:
                            candidate_makespan = max(candidate_makespan, costs[d_other])

                    if candidate_makespan < best_new_makespan - 1e-6:
                        best_new_makespan = candidate_makespan
                        best_target_d = d_target
                        best_insert_idx = idx_insert

            # Accept best improving move for this node
            if best_target_d >= 0:
                routes[d_max] = new_route_max
                routes[best_target_d] = (
                    routes[best_target_d][:best_insert_idx]
                    + [node]
                    + routes[best_target_d][best_insert_idx:]
                )
                improved = True
                break  # restart from new bottleneck computation

    # Recompute battery usage
    battery_used = [_route_battery(routes[d], graph, v0) for d in range(n_drones)]

    return routes, battery_used


# ---------------------------------------------------------------------------
# ACO Solver
# ---------------------------------------------------------------------------


class ACOSolver:
    """Ant Colony Optimisation for multi-drone area search.

    Construction heuristic: ants build complete mission plans for ALL drones
    simultaneously, using earliest-available-first drone selection and a
    per-node time reservation table to enforce anti-collision.

    Pheromone is deposited only by the iteration-best and global-best ants
    (MMAS-style elitist update) with τ_min / τ_max bounds to prevent
    premature stagnation.

    Local search includes intra-route 2-opt and inter-route relocate
    (moving nodes between drones to balance load and reduce makespan).
    """

    def __init__(
        self,
        graph: SearchGraph,
        n_drones: int,
        battery_budgets: list[float],
        alpha: float = 2.0,
        beta: float = 3.0,
        rho: float = 0.3,
        n_ants: int = 30,
        max_iter: int = 200,
        Q: float = 1.0,
        local_search: bool = True,
        seed: Optional[int] = None,
    ) -> None:
        self.graph = graph
        self.n_drones = n_drones
        self.battery_budgets = battery_budgets
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.n_ants = n_ants
        self.max_iter = max_iter
        self.Q = Q
        self.local_search = local_search
        self._rng = random.Random(seed)

        self._v0 = graph.base_node
        self._search_nodes = graph.search_nodes()
        self._n = graph.n_nodes()

        # Initialise pheromone with MMAS bounds
        L_nn = _nn_tour_length(graph)
        self._tau0 = Q / (len(self._search_nodes) * L_nn) if self._search_nodes else 1e-3
        self._tau: dict[tuple[int, int], float] = {}
        for v in graph.nodes():
            for u in graph.neighbors(v):
                self._tau[(v, u)] = self._tau0

        # MMAS pheromone bounds — prevent premature stagnation
        self._tau_max = Q / (rho * L_nn) if L_nn > 0 else 1.0
        n_search = max(len(self._search_nodes), 1)
        self._tau_min = self._tau_max / (2.0 * n_search)
        logger.debug(
            "MMAS bounds: tau_min=%.6f, tau_max=%.6f, tau0=%.6f",
            self._tau_min, self._tau_max, self._tau0,
        )

        # Precompute return-to-base shortest paths
        self._return_cost: dict[int, float] = {
            v: graph.shortest_path_length(v, self._v0) for v in graph.nodes()
        }

        self.convergence_history: list[float] = []
        self.initial_solution: Optional[Solution] = None
        self.best_iter: int = 0  # 1-based iteration where global best was found

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(
        self,
        progress_callback: Optional[callable] = None,
        progress_every: int = 1,
    ) -> Optional[Solution]:
        """Run ACO and return the best found solution.

        Parameters
        ----------
        progress_callback:
            Optional callable invoked every ``progress_every`` iterations.
            Signature: ``(iteration, total, global_best, iter_best, elapsed)``
            where all numeric values are floats/ints.
        progress_every:
            Call ``progress_callback`` once every this many iterations.
        """
        t_start = time.time()

        nn_plan = self._build_nn_solution()
        if nn_plan is not None:
            self.initial_solution = self._plan_to_solution(nn_plan, 0.0)
            self.initial_solution.solver = "NN-initial"
            logger.info("NN initial solution makespan=%.2f", self.initial_solution.makespan)

        global_best_makespan = float("inf")
        global_best_plan: Optional[dict] = None

        for iteration in range(self.max_iter):
            iter_best_makespan = float("inf")
            iter_best_plan: Optional[dict] = None

            for _ in range(self.n_ants):
                plan = self._construct_solution()
                if plan is None:
                    continue
                ms = plan["makespan"]
                if ms < iter_best_makespan:
                    iter_best_makespan = ms
                    iter_best_plan = plan

            if iter_best_plan is not None and iter_best_makespan < global_best_makespan:
                global_best_makespan = iter_best_makespan
                global_best_plan = iter_best_plan
                self.best_iter = iteration + 1

            self._update_pheromone(iter_best_plan, global_best_plan)
            self.convergence_history.append(global_best_makespan)

            logger.debug(
                "ACO iter %d/%d  best=%.2f  iter=%.2f",
                iteration + 1, self.max_iter, global_best_makespan, iter_best_makespan,
            )

            if progress_callback and (iteration + 1) % progress_every == 0:
                progress_callback(
                    iteration + 1,
                    self.max_iter,
                    global_best_makespan,
                    iter_best_makespan,
                    time.time() - t_start,
                )

        solve_time = time.time() - t_start
        logger.info(
            "ACO finished in %.1fs  global_best=%.2f  best_iter=%d/%d",
            solve_time, global_best_makespan, self.best_iter, self.max_iter,
        )

        if global_best_plan is None:
            logger.warning("ACO found no feasible solution")
            return None

        return self._plan_to_solution(global_best_plan, solve_time)

    # ------------------------------------------------------------------
    # Nearest-neighbour initial solution
    # ------------------------------------------------------------------

    def _build_nn_solution(self) -> Optional[dict]:
        """Greedy nearest-neighbour construction — no pheromone, always cheapest candidate.

        Used once before the ACO loop to produce an initial reference solution
        that is stored in ``self.initial_solution``.
        """
        unvisited = set(self._search_nodes)
        trt = TimeReservationTable()

        drone_pos = [self._v0] * self.n_drones
        drone_time = [0.0] * self.n_drones
        drone_battery_used = [0.0] * self.n_drones
        drone_routes: list[list[int]] = [[self._v0] for _ in range(self.n_drones)]
        drone_schedules: list[list[tuple[int, float, float]]] = [[] for _ in range(self.n_drones)]

        max_steps = len(self._search_nodes) * self.n_drones + 10
        for _ in range(max_steps):
            if not unvisited:
                break

            d = int(min(range(self.n_drones), key=lambda k: drone_time[k]))
            pos = drone_pos[d]
            t_now = drone_time[d]
            budget = self.battery_budgets[d]
            used = drone_battery_used[d]

            # Pick the cheapest reachable unvisited node (pure greedy)
            best_cost = float("inf")
            best_node = None
            best_arrive = 0.0
            for v in unvisited:
                if not self.graph.has_edge(pos, v):
                    continue
                tt = self.graph.travel_time(pos, v)
                st = self.graph.search_time(v)
                ret = self._return_cost[v]
                if used + tt + st + ret > budget + 1e-6:
                    continue
                t_arrive = trt.earliest_free(v, st, t_now + tt)
                cost = t_arrive + st  # earliest finish time as tiebreaker
                if cost < best_cost:
                    best_cost = cost
                    best_node = v
                    best_arrive = t_arrive

            if best_node is None:
                drone_time[d] = t_now + self._return_cost[pos]
                drone_pos[d] = self._v0
                drone_routes[d].append(self._v0)
                if not unvisited:
                    break
                drone_time[d] = float("inf")
                continue

            tt = self.graph.travel_time(pos, best_node)
            st = self.graph.search_time(best_node)
            t_depart = best_arrive + st

            trt.reserve(best_node, best_arrive, t_depart)
            unvisited.discard(best_node)

            drone_battery_used[d] += tt + st
            drone_pos[d] = best_node
            drone_time[d] = t_depart
            drone_routes[d].append(best_node)
            drone_schedules[d].append((best_node, best_arrive, t_depart))

        makespan = 0.0
        for d in range(self.n_drones):
            if drone_time[d] == float("inf"):
                drone_time[d] = 0.0
            ret_time = drone_time[d] + self._return_cost[drone_pos[d]]
            drone_battery_used[d] += self._return_cost[drone_pos[d]]
            drone_routes[d].append(self._v0)
            makespan = max(makespan, ret_time)

        if unvisited:
            return None

        return {
            "routes": {d: drone_routes[d] for d in range(self.n_drones)},
            "schedules": {d: drone_schedules[d] for d in range(self.n_drones)},
            "battery_used": {d: drone_battery_used[d] for d in range(self.n_drones)},
            "makespan": makespan,
        }

    # ------------------------------------------------------------------
    # Ant construction
    # ------------------------------------------------------------------

    def _construct_solution(self) -> Optional[dict]:
        """Build a complete mission plan for all drones (one ant)."""
        unvisited = set(self._search_nodes)
        trt = TimeReservationTable()

        # State per drone
        drone_pos = [self._v0] * self.n_drones
        drone_time = [0.0] * self.n_drones
        drone_battery_used = [0.0] * self.n_drones
        drone_routes: list[list[int]] = [[self._v0] for _ in range(self.n_drones)]
        drone_schedules: list[list[tuple[int, float, float]]] = [[] for _ in range(self.n_drones)]

        # Repeat until all nodes assigned
        max_steps = len(self._search_nodes) * self.n_drones + 10
        for _ in range(max_steps):
            if not unvisited:
                break

            # Earliest-available-first drone selection
            d = int(min(range(self.n_drones), key=lambda k: drone_time[k]))
            pos = drone_pos[d]
            t_now = drone_time[d]
            budget = self.battery_budgets[d]
            used = drone_battery_used[d]

            # Build candidate list
            candidates: list[tuple[int, float]] = []
            for v in unvisited:
                if not self.graph.has_edge(pos, v):
                    continue
                tt = self.graph.travel_time(pos, v)
                st = self.graph.search_time(v)
                ret = self._return_cost[v]
                # Battery check
                if used + tt + st + ret > budget + 1e-6:
                    continue
                # Earliest arrival (after any existing reservation)
                t_arrive = t_now + tt
                t_arrive = trt.earliest_free(v, st, t_arrive)
                # Heuristic
                eta = 1.0 / max(tt + st, 1e-6)
                tau = self._tau.get((pos, v), self._tau0)
                prob_weight = (tau ** self.alpha) * (eta ** self.beta)
                candidates.append((v, prob_weight))

            if not candidates:
                # No feasible next node → drone returns to base
                drone_time[d] = t_now + self._return_cost[pos]
                drone_pos[d] = self._v0
                drone_routes[d].append(self._v0)
                # Mark drone as "done" by setting its time very high if no nodes remain
                if not unvisited:
                    break
                # Force other drones; bump this drone's time so it isn't picked again
                drone_time[d] = float("inf")
                continue

            # Probabilistic selection
            v_chosen = self._select(candidates)

            tt = self.graph.travel_time(pos, v_chosen)
            st = self.graph.search_time(v_chosen)
            t_arrive = t_now + tt
            t_arrive = trt.earliest_free(v_chosen, st, t_arrive)
            t_depart = t_arrive + st

            trt.reserve(v_chosen, t_arrive, t_depart)
            unvisited.discard(v_chosen)

            drone_battery_used[d] += tt + st
            drone_pos[d] = v_chosen
            drone_time[d] = t_depart
            drone_routes[d].append(v_chosen)
            drone_schedules[d].append((v_chosen, t_arrive, t_depart))

        # All drones return to base
        makespan = 0.0
        for d in range(self.n_drones):
            if drone_time[d] == float("inf"):
                drone_time[d] = 0.0  # drone never left meaningfully
            ret_time = drone_time[d] + self._return_cost[drone_pos[d]]
            drone_battery_used[d] += self._return_cost[drone_pos[d]]
            drone_routes[d].append(self._v0)
            makespan = max(makespan, ret_time)

        if unvisited:
            # Could not cover all nodes
            return None

        # Optional local search: intra-route 2-opt + inter-route relocate
        if self.local_search:
            for d in range(self.n_drones):
                drone_routes[d] = _two_opt(drone_routes[d], self.graph)
            # Inter-route relocate: try moving nodes between drones
            drone_routes, drone_battery_used = _inter_route_relocate(
                drone_routes, self.graph, self.battery_budgets,
                self._return_cost, self._v0, self.n_drones,
            )

        # Recompute makespan after local search
        makespan = 0.0
        for d in range(self.n_drones):
            route = drone_routes[d]
            t = 0.0
            for k in range(len(route) - 1):
                t += self.graph.shortest_path_length(route[k], route[k + 1])
                if route[k + 1] != self._v0:
                    t += self.graph.search_time(route[k + 1])
            makespan = max(makespan, t)

        return {
            "routes": {d: drone_routes[d] for d in range(self.n_drones)},
            "schedules": {d: drone_schedules[d] for d in range(self.n_drones)},
            "battery_used": {d: drone_battery_used[d] for d in range(self.n_drones)},
            "makespan": makespan,
        }

    def _select(self, candidates: list[tuple[int, float]]) -> int:
        """Roulette-wheel selection over (node, weight) pairs."""
        total = sum(w for _, w in candidates)
        if total <= 0:
            return self._rng.choice([v for v, _ in candidates])
        r = self._rng.uniform(0, total)
        cumulative = 0.0
        for v, w in candidates:
            cumulative += w
            if cumulative >= r:
                return v
        return candidates[-1][0]

    # ------------------------------------------------------------------
    # Pheromone update
    # ------------------------------------------------------------------

    def _update_pheromone(
        self,
        iter_best: Optional[dict],
        global_best: Optional[dict],
    ) -> None:
        # Evaporation
        for key in self._tau:
            self._tau[key] *= (1 - self.rho)

        # Deposit: iteration-best + global-best (MMAS-style)
        for plan in (iter_best, global_best):
            if plan is None:
                continue
            delta = self.Q / max(plan["makespan"], 1e-6)
            for d, route in plan["routes"].items():
                for k in range(len(route) - 1):
                    edge = (route[k], route[k + 1])
                    self._tau[edge] = self._tau.get(edge, self._tau0) + delta

        # MMAS bounds clamping
        for key in self._tau:
            self._tau[key] = max(min(self._tau[key], self._tau_max), self._tau_min)

    # ------------------------------------------------------------------
    # Plan → Solution
    # ------------------------------------------------------------------

    def _plan_to_solution(self, plan: dict, solve_time: float) -> Solution:
        g = self.graph
        total_flight = 0.0
        for d, route in plan["routes"].items():
            for k in range(len(route) - 1):
                if g.has_edge(route[k], route[k + 1]):
                    total_flight += g.travel_time(route[k], route[k + 1])

        # Rebuild schedules from routes to ensure timing is consistent
        schedules: dict[int, list[tuple[int, float, float]]] = {}
        trt = TimeReservationTable()
        v0 = self._v0

        for d in range(self.n_drones):
            route = plan["routes"][d]
            t = 0.0
            sched: list[tuple[int, float, float]] = []
            prev = v0
            for v in route[1:]:
                if v == v0:
                    break
                if not g.has_edge(prev, v):
                    tt = g.shortest_path_length(prev, v)
                else:
                    tt = g.travel_time(prev, v)
                t_arrive = t + tt
                st = g.search_time(v)
                t_arrive = trt.earliest_free(v, st, t_arrive)
                t_depart = t_arrive + st
                trt.reserve(v, t_arrive, t_depart)
                sched.append((v, t_arrive, t_depart))
                t = t_depart
                prev = v
            schedules[d] = sched

        # Recompute makespan from schedules
        makespan = 0.0
        for d in range(self.n_drones):
            route = plan["routes"][d]
            sched = schedules[d]
            if sched:
                last_v, _, last_dep = sched[-1]
                makespan = max(makespan, last_dep + g.shortest_path_length(last_v, v0))
            else:
                makespan = max(makespan, 0.0)

        return Solution(
            routes=plan["routes"],
            schedule=schedules,
            makespan=makespan,
            total_flight_time=total_flight,
            battery_usage={d: plan["battery_used"][d] for d in range(self.n_drones)},
            solver="ACO",
            solve_time=solve_time,
        )
