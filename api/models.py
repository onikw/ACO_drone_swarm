"""Pydantic request/response models for the drone search API."""
from __future__ import annotations

from typing import Any, Optional
from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# Graph models
# ---------------------------------------------------------------------------


class NodeModel(BaseModel):
    id: int
    search_time: float = Field(..., gt=0, description="Search time in minutes")


class EdgeModel(BaseModel):
    i: int
    j: int
    travel_time: float = Field(..., gt=0, description="Travel time in minutes")


class GraphModel(BaseModel):
    base_node: int = 0
    nodes: list[NodeModel]
    edges: list[EdgeModel]


# ---------------------------------------------------------------------------
# Generator request models
# ---------------------------------------------------------------------------


class GridGraphRequest(BaseModel):
    rows: int = Field(2, ge=1)
    cols: int = Field(3, ge=1)
    search_time_range: tuple[float, float] = (5.0, 30.0)
    travel_time_range: tuple[float, float] = (1.0, 5.0)
    base_node: int = 0
    seed: Optional[int] = None


class RandomGraphRequest(BaseModel):
    n: int = Field(10, ge=2)
    density: float = Field(0.4, ge=0.1, le=1.0)
    search_time_range: tuple[float, float] = (5.0, 30.0)
    travel_time_range: tuple[float, float] = (1.0, 5.0)
    base_node: int = 0
    seed: Optional[int] = None


class ClusterGraphRequest(BaseModel):
    n_clusters: int = Field(3, ge=1)
    nodes_per_cluster: int = Field(4, ge=2)
    search_time_range: tuple[float, float] = (5.0, 30.0)
    travel_time_range_intra: tuple[float, float] = (1.0, 3.0)
    travel_time_range_inter: tuple[float, float] = (3.0, 8.0)
    base_node: int = 0
    seed: Optional[int] = None


class GenerateGraphRequest(BaseModel):
    type: str = Field(..., description="'grid', 'random', or 'cluster'")
    params: dict[str, Any] = Field(default_factory=dict)


# ---------------------------------------------------------------------------
# Solver request models
# ---------------------------------------------------------------------------


class MILPRequest(BaseModel):
    graph: GraphModel
    n_drones: int = Field(..., ge=1)
    battery_budgets: list[float] = Field(..., min_length=1)
    time_limit: int = Field(300, ge=10)


class ACORequest(BaseModel):
    graph: GraphModel
    n_drones: int = Field(..., ge=1)
    battery_budgets: list[float] = Field(..., min_length=1)
    alpha: float = 2.0
    beta: float = 3.0
    rho: float = 0.3
    n_ants: int = Field(30, ge=1)
    max_iter: int = Field(200, ge=1)
    local_search: bool = True
    seed: Optional[int] = None


# ---------------------------------------------------------------------------
# Solution and validate models
# ---------------------------------------------------------------------------


class ScheduleEntry(BaseModel):
    node: int
    arrive_time: float
    depart_time: float


class SolutionResponse(BaseModel):
    routes: dict[str, list[int]]
    schedule: dict[str, list[ScheduleEntry]]
    makespan: float
    total_flight_time: float
    battery_usage: dict[str, float]
    solver: str
    solve_time: float
    violations: list[str] = Field(default_factory=list)


class ValidateRequest(BaseModel):
    solution: dict[str, Any]
    graph: GraphModel
    n_drones: int
    battery_budgets: list[float]
