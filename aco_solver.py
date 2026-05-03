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
# ACO Solver
# ---------------------------------------------------------------------------


class ACOSolver:
    """Ant Colony Optimisation for multi-drone area search.

    Construction heuristic: ants build complete mission plans for ALL drones
    simultaneously, using earliest-available-first drone selection and a
    per-node time reservation table to enforce anti-collision.

    Pheromone is deposited only by the iteration-best and global-best ants
    (MMAS-style elitist update).
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

        # Initialise pheromone
        L_nn = _nn_tour_length(graph)
        self._tau0 = Q / (len(self._search_nodes) * L_nn) if self._search_nodes else 1e-3
        self._tau: dict[tuple[int, int], float] = {}
        for v in graph.nodes():
            for u in graph.neighbors(v):
                self._tau[(v, u)] = self._tau0

        # Precompute return-to-base shortest paths
        self._return_cost: dict[int, float] = {
            v: graph.shortest_path_length(v, self._v0) for v in graph.nodes()
        }

        self.convergence_history: list[float] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(self) -> Optional[Solution]:
        """Run ACO and return the best found solution."""
        t_start = time.time()

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

            self._update_pheromone(iter_best_plan, global_best_plan)
            self.convergence_history.append(global_best_makespan)

            if iteration % 20 == 0:
                logger.info(
                    "ACO iter %d/%d  best_makespan=%.2f",
                    iteration,
                    self.max_iter,
                    global_best_makespan,
                )

        solve_time = time.time() - t_start
        logger.info(
            "ACO finished in %.1fs  global_best_makespan=%.2f", solve_time, global_best_makespan
        )

        if global_best_plan is None:
            logger.warning("ACO found no feasible solution")
            return None

        return self._plan_to_solution(global_best_plan, solve_time)

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

        # Optional 2-opt local search
        if self.local_search:
            for d in range(self.n_drones):
                drone_routes[d] = _two_opt(drone_routes[d], self.graph)

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
            self._tau[key] = max(self._tau[key] * (1 - self.rho), 1e-10)

        # Deposit: iteration-best + global-best (MMAS-style)
        for plan in (iter_best, global_best):
            if plan is None:
                continue
            delta = self.Q / max(plan["makespan"], 1e-6)
            for d, route in plan["routes"].items():
                for k in range(len(route) - 1):
                    edge = (route[k], route[k + 1])
                    self._tau[edge] = self._tau.get(edge, self._tau0) + delta

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
