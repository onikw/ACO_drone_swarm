"""MILP solver for multi-drone area search using PuLP (CBC backend)."""
from __future__ import annotations

import itertools
import logging
import time
from typing import Optional

import pulp

from graph import SearchGraph
from solution import Solution

logger = logging.getLogger(__name__)

_BIG_M = 10_000.0  # large-M for indicator constraints


class MILPSolver:
    """Exact MILP solver for the multi-drone search scheduling problem.

    Decision variables
    ------------------
    x[i,j,d]  ∈ {0,1}  — drone d traverses edge (i→j)
    y[v,d]    ∈ {0,1}  — drone d searches node v
    a[v,d]    ≥ 0      — arrival time of drone d at node v
    u[v,d]    ≥ 0      — MTZ subtour-elimination variable
    z[v,d1,d2] ∈{0,1}  — ordering of drones d1, d2 at shared node v
    C_max     ≥ 0      — makespan (objective)

    transit_allowed
    ---------------
    When True (default), drones may fly through nodes they do not search
    ("transit").  Flow conservation becomes a balance constraint instead of
    equality to y, the x-y consistency constraint (spec §4) is dropped, and
    the temporal / makespan constraints account for search time only when the
    drone actually searches a node.  This matches the ACO's relaxed model and
    typically yields shorter makespans on sparse graphs.

    When False, the strict "transit = search" formulation is used: every
    traversed node is also searched, exactly as written in the spec.
    """

    def __init__(
        self,
        graph: SearchGraph,
        n_drones: int,
        battery_budgets: list[float],
        transit_allowed: bool = True,
    ) -> None:
        self.graph = graph
        self.n_drones = n_drones
        self.battery_budgets = battery_budgets
        self.transit_allowed = transit_allowed

        self._V = graph.nodes()
        self._V_prime = graph.search_nodes()
        self._D = list(range(n_drones))
        self._v0 = graph.base_node
        self._n = len(self._V)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(self, time_limit: int = 300) -> Optional[Solution]:
        """Build and solve the MILP. Returns None if no feasible solution found."""
        t_start = time.time()
        prob, vars_ = self._build_model()

        solver = pulp.PULP_CBC_CMD(
            timeLimit=time_limit,
            msg=1,
            gapRel=0.0,
        )
        mode = "transit-allowed" if self.transit_allowed else "transit=search"
        logger.info("Starting CBC solver [%s] (time limit %ds)…", mode, time_limit)
        prob.solve(solver)
        solve_time = time.time() - t_start

        status = pulp.LpStatus[prob.status]
        logger.info("CBC status: %s  (%.1fs)", status, solve_time)

        if pulp.value(vars_["C_max"]) is None:
            logger.warning("MILP: no incumbent found (status=%s)", status)
            return None

        try:
            sol = self._extract_solution(vars_, solve_time)
        except Exception as exc:
            logger.error("Could not extract solution: %s", exc)
            return None

        return sol

    # ------------------------------------------------------------------
    # Model construction
    # ------------------------------------------------------------------

    def _build_model(self):
        prob = pulp.LpProblem("DroneSearch", pulp.LpMinimize)

        V = self._V
        Vp = self._V_prime
        D = self._D
        v0 = self._v0
        n = self._n
        g = self.graph
        ta = self.transit_allowed

        # Directed edges that exist in the graph
        edges = [(i, j) for i in V for j in V if i != j and g.has_edge(i, j)]

        # ---- Decision variables ----------------------------------------
        x = {
            (i, j, d): pulp.LpVariable(f"x_{i}_{j}_{d}", cat="Binary")
            for (i, j) in edges
            for d in D
        }
        y = {
            (v, d): pulp.LpVariable(f"y_{v}_{d}", cat="Binary")
            for v in Vp
            for d in D
        }
        a = {
            (v, d): pulp.LpVariable(f"a_{v}_{d}", lowBound=0)
            for v in V
            for d in D
        }
        u = {
            (v, d): pulp.LpVariable(f"u_{v}_{d}", lowBound=0, upBound=n)
            for v in Vp
            for d in D
        }
        C_max = pulp.LpVariable("C_max", lowBound=0)

        drone_pairs = list(itertools.combinations(D, 2))
        z = {
            (v, d1, d2): pulp.LpVariable(f"z_{v}_{d1}_{d2}", cat="Binary")
            for v in Vp
            for (d1, d2) in drone_pairs
        }

        # ---- Objective -------------------------------------------------
        prob += C_max, "Minimize_makespan"

        # ---- Constraint 1: Coverage ------------------------------------
        for v in Vp:
            prob += pulp.lpSum(y[v, d] for d in D) == 1, f"cov_{v}"

        # ---- Constraints 2 & 3: Flow conservation ----------------------
        for d in D:
            # Base: at most one trip; outgoing == incoming.
            prob += (
                pulp.lpSum(x[v0, j, d] for (i, j) in edges if i == v0) <= 1,
                f"flow_base_out_{d}",
            )
            prob += (
                pulp.lpSum(x[v0, j, d] for (i, j) in edges if i == v0)
                == pulp.lpSum(x[i, v0, d] for (i, j) in edges if j == v0),
                f"flow_base_balance_{d}",
            )

            for v in Vp:
                flow_in = pulp.lpSum(x[i, v, d] for (i, j) in edges if j == v)
                flow_out = pulp.lpSum(x[v, j, d] for (i, j) in edges if i == v)

                if ta:
                    # Transit-allowed: flow is balanced and at most 1;
                    # drone must ENTER v to search it (but need not search every node it enters).
                    prob += flow_in == flow_out, f"flow_balance_{v}_{d}"
                    prob += flow_in <= 1, f"flow_in_leq1_{v}_{d}"
                    prob += y[v, d] <= flow_in, f"search_requires_visit_{v}_{d}"
                else:
                    # Strict transit=search: flow exactly equals y (spec §2/3).
                    prob += flow_in == y[v, d], f"flow_in_{v}_{d}"
                    prob += flow_out == y[v, d], f"flow_out_{v}_{d}"

        # ---- Constraint 4: x-y consistency (spec §4) -------------------
        # Redundant when transit_allowed (flow conservation already handles it
        # in the strict case; in the relaxed case it would forbid transit).
        if not ta:
            for (i, j) in edges:
                for d in D:
                    if i != v0:
                        prob += x[i, j, d] <= y[i, d], f"xy_src_{i}_{j}_{d}"
                    if j != v0:
                        prob += x[i, j, d] <= y[j, d], f"xy_dst_{i}_{j}_{d}"

        # ---- Constraint 5: MTZ subtour elimination ---------------------
        for d in D:
            for (i, j) in edges:
                if i in Vp and j in Vp:
                    prob += (
                        u[i, d] - u[j, d] + n * x[i, j, d] <= n - 1,
                        f"mtz_{i}_{j}_{d}",
                    )

        # ---- Constraint 6: Battery budget ------------------------------
        for d in D:
            budget = self.battery_budgets[d]
            travel_expr = pulp.lpSum(
                g.travel_time(i, j) * x[i, j, d] for (i, j) in edges
            )
            search_expr = pulp.lpSum(g.search_time(v) * y[v, d] for v in Vp)
            prob += travel_expr + search_expr <= budget, f"battery_{d}"

        # ---- Constraint 7: Temporal consistency ------------------------
        # When transit_allowed, search time at i is included only if drone searches i.
        for d in D:
            for (i, j) in edges:
                if i in Vp and j in Vp:
                    if ta:
                        # s(i) * y[i,d]: linear because s(i) is a constant.
                        search_at_i = g.search_time(i) * y[i, d]
                    else:
                        search_at_i = g.search_time(i)
                    prob += (
                        a[j, d]
                        >= a[i, d]
                        + search_at_i
                        + g.travel_time(i, j)
                        - _BIG_M * (1 - x[i, j, d]),
                        f"time_vv_{i}_{j}_{d}",
                    )
                elif i == v0 and j in Vp:
                    prob += (
                        a[j, d]
                        >= g.travel_time(v0, j) - _BIG_M * (1 - x[v0, j, d]),
                        f"time_base_{j}_{d}",
                    )

        # ---- Constraint 9: Anti-collision ------------------------------
        for v in Vp:
            for d1, d2 in drone_pairs:
                prob += (
                    a[v, d1] + g.search_time(v)
                    <= a[v, d2]
                    + _BIG_M * (1 - z[v, d1, d2])
                    + _BIG_M * (2 - y[v, d1] - y[v, d2]),
                    f"anticol_12_{v}_{d1}_{d2}",
                )
                prob += (
                    a[v, d2] + g.search_time(v)
                    <= a[v, d1]
                    + _BIG_M * z[v, d1, d2]
                    + _BIG_M * (2 - y[v, d1] - y[v, d2]),
                    f"anticol_21_{v}_{d1}_{d2}",
                )

        # ---- Constraint 10: Makespan -----------------------------------
        for d in D:
            for v in Vp:
                if g.has_edge(v, v0):
                    if ta:
                        search_at_v = g.search_time(v) * y[v, d]
                    else:
                        search_at_v = g.search_time(v)
                    prob += (
                        C_max
                        >= a[v, d]
                        + search_at_v
                        + g.travel_time(v, v0) * x[v, v0, d],
                        f"makespan_{v}_{d}",
                    )

        logger.info(
            "Model built: %d vars, %d constraints",
            len(prob.variables()),
            len(prob.constraints),
        )
        return prob, {"x": x, "y": y, "a": a, "u": u, "z": z, "C_max": C_max}

    # ------------------------------------------------------------------
    # Solution extraction
    # ------------------------------------------------------------------

    def _extract_solution(self, vars_: dict, solve_time: float) -> Solution:
        x, y, a, C_max = vars_["x"], vars_["y"], vars_["a"], vars_["C_max"]
        g = self.graph
        v0 = self._v0
        Vp = self._V_prime
        D = self._D
        edges = [(i, j) for i in self._V for j in self._V if i != j and g.has_edge(i, j)]

        def val(var) -> float:
            v = pulp.value(var)
            return v if v is not None else 0.0

        makespan = val(C_max)
        solver_name = "MILP-CBC" + ("-transit" if self.transit_allowed else "-strict")

        routes: dict[int, list[int]] = {}
        schedule: dict[int, list[tuple[int, float, float]]] = {}
        battery_usage: dict[int, float] = {}
        total_flight = 0.0

        for d in D:
            active_edges = {
                (i, j) for (i, j) in edges if val(x.get((i, j, d), 0)) > 0.5
            }
            route = _reconstruct_route(v0, active_edges)
            routes[d] = route

            # Schedule: only nodes actually searched (y[v,d] = 1)
            searched = {v for v in Vp if val(y.get((v, d), 0)) > 0.5}
            sched: list[tuple[int, float, float]] = []
            for v in route:
                if v == v0 or v not in searched:
                    continue
                arrive = val(a[v, d])
                depart = arrive + g.search_time(v)
                sched.append((v, arrive, depart))
            sched.sort(key=lambda t: t[1])
            schedule[d] = sched

            travel = sum(g.travel_time(i, j) for (i, j) in active_edges)
            search = sum(g.search_time(v) for v in searched)
            battery_usage[d] = travel + search
            total_flight += travel

        return Solution(
            routes=routes,
            schedule=schedule,
            makespan=makespan,
            total_flight_time=total_flight,
            battery_usage=battery_usage,
            solver=solver_name,
            solve_time=solve_time,
        )


def _reconstruct_route(start: int, edges: set[tuple[int, int]]) -> list[int]:
    """Walk the active-edge set to build an ordered route starting and ending at start."""
    if not edges:
        return [start, start]

    adj: dict[int, list[int]] = {}
    for i, j in edges:
        adj.setdefault(i, []).append(j)

    route = [start]
    current = start
    visited_edges: set[tuple[int, int]] = set()
    for _ in range(len(edges) + 1):
        nexts = adj.get(current, [])
        nxt = next((j for j in nexts if (current, j) not in visited_edges), None)
        if nxt is None:
            break
        visited_edges.add((current, nxt))
        route.append(nxt)
        current = nxt
        if current == start:
            break

    if route[-1] != start:
        route.append(start)
    return route
