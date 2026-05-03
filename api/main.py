"""FastAPI REST API for drone search planning."""
from __future__ import annotations

import logging
import sys
import os

# Allow importing root-level modules when running from api/ directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse

from graph import SearchGraph
from graph_generator import generate_grid_graph, generate_random_graph, generate_cluster_graph
from milp_solver import MILPSolver
from aco_solver import ACOSolver
from solution import Solution
from constraints import validate_solution

from api.models import (
    GenerateGraphRequest, GraphModel,
    MILPRequest, ACORequest, ValidateRequest,
    SolutionResponse, ScheduleEntry,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Drone Search Planning API",
    description="Multi-drone area search scheduling via MILP and ACO",
    version="1.0.0",
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _graph_from_model(gm: GraphModel) -> SearchGraph:
    g = SearchGraph(base_node=gm.base_node)
    for node in gm.nodes:
        g.add_node(node.id, node.search_time)
    for edge in gm.edges:
        g.add_edge(edge.i, edge.j, edge.travel_time)
    return g


def _solution_to_response(sol: Solution, graph: SearchGraph, n_drones: int, budgets: list[float]) -> SolutionResponse:
    violations = validate_solution(sol, graph, n_drones, budgets)
    schedule_out: dict[str, list[ScheduleEntry]] = {}
    for d, entries in sol.schedule.items():
        schedule_out[str(d)] = [
            ScheduleEntry(node=node, arrive_time=arr, depart_time=dep)
            for node, arr, dep in entries
        ]
    return SolutionResponse(
        routes={str(k): v for k, v in sol.routes.items()},
        schedule=schedule_out,
        makespan=sol.makespan,
        total_flight_time=sol.total_flight_time,
        battery_usage={str(k): v for k, v in sol.battery_usage.items()},
        solver=sol.solver,
        solve_time=sol.solve_time,
        violations=violations,
    )


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@app.get("/health")
def health() -> dict:
    return {"status": "ok"}


@app.post("/graph/generate", response_model=GraphModel)
def generate_graph(request: GenerateGraphRequest) -> GraphModel:
    """Generate a test graph."""
    t = request.type.lower()
    p = request.params
    try:
        if t == "grid":
            g = generate_grid_graph(
                rows=p.get("rows", 2),
                cols=p.get("cols", 3),
                search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
                travel_time_range=tuple(p.get("travel_time_range", [1.0, 5.0])),
                base_node=p.get("base_node", 0),
                seed=p.get("seed"),
            )
        elif t == "random":
            g = generate_random_graph(
                n=p.get("n", 10),
                density=p.get("density", 0.4),
                search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
                travel_time_range=tuple(p.get("travel_time_range", [1.0, 5.0])),
                base_node=p.get("base_node", 0),
                seed=p.get("seed"),
            )
        elif t == "cluster":
            g = generate_cluster_graph(
                n_clusters=p.get("n_clusters", 3),
                nodes_per_cluster=p.get("nodes_per_cluster", 4),
                search_time_range=tuple(p.get("search_time_range", [5.0, 30.0])),
                travel_time_range_intra=tuple(p.get("travel_time_range_intra", [1.0, 3.0])),
                travel_time_range_inter=tuple(p.get("travel_time_range_inter", [3.0, 8.0])),
                base_node=p.get("base_node", 0),
                seed=p.get("seed"),
            )
        else:
            raise HTTPException(status_code=400, detail=f"Unknown graph type: {t!r}")
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))

    d = g.to_dict()
    return GraphModel(
        base_node=d["base_node"],
        nodes=[{"id": n["id"], "search_time": n["search_time"]} for n in d["nodes"]],
        edges=[{"i": e["i"], "j": e["j"], "travel_time": e["travel_time"]} for e in d["edges"]],
    )


@app.post("/solve/milp", response_model=SolutionResponse)
def solve_milp(request: MILPRequest) -> SolutionResponse:
    """Solve with MILP (CBC). Suitable for small instances (≤12 nodes)."""
    graph = _graph_from_model(request.graph)
    if len(request.battery_budgets) < request.n_drones:
        raise HTTPException(status_code=400, detail="Not enough battery budgets for drones")
    solver = MILPSolver(graph, request.n_drones, request.battery_budgets)
    sol = solver.solve(time_limit=request.time_limit)
    if sol is None:
        raise HTTPException(status_code=422, detail="MILP found no feasible solution")
    return _solution_to_response(sol, graph, request.n_drones, request.battery_budgets)


@app.post("/solve/aco", response_model=SolutionResponse)
def solve_aco(request: ACORequest) -> SolutionResponse:
    """Solve with ACO metaheuristic. Scales to larger instances."""
    graph = _graph_from_model(request.graph)
    if len(request.battery_budgets) < request.n_drones:
        raise HTTPException(status_code=400, detail="Not enough battery budgets for drones")
    solver = ACOSolver(
        graph,
        request.n_drones,
        request.battery_budgets,
        alpha=request.alpha,
        beta=request.beta,
        rho=request.rho,
        n_ants=request.n_ants,
        max_iter=request.max_iter,
        local_search=request.local_search,
        seed=request.seed,
    )
    sol = solver.solve()
    if sol is None:
        raise HTTPException(status_code=422, detail="ACO found no feasible solution")
    return _solution_to_response(sol, graph, request.n_drones, request.battery_budgets)


@app.post("/validate")
def validate(request: ValidateRequest) -> dict:
    """Validate a solution dict against the graph and constraints."""
    graph = _graph_from_model(request.graph)
    try:
        sol = Solution.from_dict(request.solution)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid solution format: {exc}")
    violations = validate_solution(sol, graph, request.n_drones, request.battery_budgets)
    return {"feasible": len(violations) == 0, "violations": violations}
