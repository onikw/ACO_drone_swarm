"""API endpoint tests using FastAPI TestClient."""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from fastapi.testclient import TestClient

from api.main import app

client = TestClient(app)

# ---------------------------------------------------------------------------
# Shared small graph payload
# ---------------------------------------------------------------------------

SMALL_GRAPH = {
    "base_node": 0,
    "nodes": [
        {"id": 0, "search_time": 1.0},
        {"id": 1, "search_time": 10.0},
        {"id": 2, "search_time": 10.0},
        {"id": 3, "search_time": 10.0},
    ],
    "edges": [
        {"i": 0, "j": 1, "travel_time": 2.0},
        {"i": 0, "j": 2, "travel_time": 2.0},
        {"i": 0, "j": 3, "travel_time": 2.0},
        {"i": 1, "j": 2, "travel_time": 2.0},
        {"i": 2, "j": 3, "travel_time": 2.0},
    ],
}


# ---------------------------------------------------------------------------
# Health
# ---------------------------------------------------------------------------


def test_health():
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"


# ---------------------------------------------------------------------------
# Graph generation
# ---------------------------------------------------------------------------


def test_generate_grid_graph():
    r = client.post("/graph/generate", json={"type": "grid", "params": {"rows": 2, "cols": 3, "seed": 42}})
    assert r.status_code == 200
    data = r.json()
    assert len(data["nodes"]) == 6
    assert data["base_node"] == 0


def test_generate_random_graph():
    r = client.post("/graph/generate", json={"type": "random", "params": {"n": 8, "seed": 1}})
    assert r.status_code == 200
    assert len(r.json()["nodes"]) == 8


def test_generate_cluster_graph():
    r = client.post("/graph/generate", json={"type": "cluster", "params": {"n_clusters": 2, "nodes_per_cluster": 3}})
    assert r.status_code == 200
    assert len(r.json()["nodes"]) == 6


def test_generate_unknown_type():
    r = client.post("/graph/generate", json={"type": "hexagonal", "params": {}})
    assert r.status_code == 400


# ---------------------------------------------------------------------------
# MILP solver
# ---------------------------------------------------------------------------


def test_solve_milp_basic():
    payload = {
        "graph": SMALL_GRAPH,
        "n_drones": 2,
        "battery_budgets": [60.0, 60.0],
        "time_limit": 60,
    }
    r = client.post("/solve/milp", json=payload)
    assert r.status_code == 200
    data = r.json()
    assert "makespan" in data
    assert data["makespan"] > 0
    assert data["violations"] == []


def test_solve_milp_missing_budget():
    payload = {
        "graph": SMALL_GRAPH,
        "n_drones": 3,
        "battery_budgets": [60.0],  # only 1 budget for 3 drones
        "time_limit": 60,
    }
    r = client.post("/solve/milp", json=payload)
    assert r.status_code == 400


# ---------------------------------------------------------------------------
# ACO solver
# ---------------------------------------------------------------------------


def test_solve_aco_basic():
    payload = {
        "graph": SMALL_GRAPH,
        "n_drones": 2,
        "battery_budgets": [60.0, 60.0],
        "n_ants": 10,
        "max_iter": 20,
        "seed": 42,
    }
    r = client.post("/solve/aco", json=payload)
    assert r.status_code == 200
    data = r.json()
    assert "makespan" in data
    assert data["makespan"] > 0


def test_solve_aco_missing_budget():
    payload = {
        "graph": SMALL_GRAPH,
        "n_drones": 3,
        "battery_budgets": [60.0],
        "n_ants": 5,
        "max_iter": 5,
    }
    r = client.post("/solve/aco", json=payload)
    assert r.status_code == 400


# ---------------------------------------------------------------------------
# Validate endpoint
# ---------------------------------------------------------------------------


def test_validate_endpoint_with_aco_solution():
    # First generate a solution
    solve_payload = {
        "graph": SMALL_GRAPH,
        "n_drones": 2,
        "battery_budgets": [60.0, 60.0],
        "n_ants": 10,
        "max_iter": 20,
        "seed": 0,
    }
    sol_r = client.post("/solve/aco", json=solve_payload)
    assert sol_r.status_code == 200
    sol_data = sol_r.json()

    # Convert response to Solution dict format expected by /validate
    solution_dict = {
        "routes": sol_data["routes"],
        "schedule": {
            k: [(e["node"], e["arrive_time"], e["depart_time"]) for e in v]
            for k, v in sol_data["schedule"].items()
        },
        "makespan": sol_data["makespan"],
        "total_flight_time": sol_data["total_flight_time"],
        "battery_usage": sol_data["battery_usage"],
        "solver": sol_data["solver"],
        "solve_time": sol_data["solve_time"],
    }
    validate_payload = {
        "solution": solution_dict,
        "graph": SMALL_GRAPH,
        "n_drones": 2,
        "battery_budgets": [60.0, 60.0],
    }
    r = client.post("/validate", json=validate_payload)
    assert r.status_code == 200
    result = r.json()
    assert result["feasible"] is True
    assert result["violations"] == []
