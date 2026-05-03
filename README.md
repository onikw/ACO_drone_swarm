# Multi-Drone Area Search Scheduler

Optimal and heuristic scheduling of multiple drones for area search, with anti-collision guarantees.

## Installation

```bash
pip install -r requirements.txt
```

## Quick start

```bash
# Small example (MILP + ACO, 2x3 grid)
python examples/small_example.py

# Medium example (ACO only, 20 nodes)
python examples/medium_example.py

# Scaling comparison
python examples/compare_solvers.py
```

## REST API

```bash
uvicorn api.main:app --reload
# Swagger UI: http://localhost:8000/docs
```

### Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/health` | Health check |
| POST | `/graph/generate` | Generate a test graph (`grid`, `random`, `cluster`) |
| POST | `/solve/milp` | Solve with MILP/CBC (small instances, ≤12 nodes) |
| POST | `/solve/aco` | Solve with ACO (scales to 20+ nodes) |
| POST | `/validate` | Validate a solution against all constraints |

## Tests

```bash
pytest tests/ -v
```

## Project structure

```
graph.py              # SearchGraph model (networkx)
graph_generator.py    # Grid / random / cluster graph generators
solution.py           # Solution dataclass + serialisation
constraints.py        # Feasibility validator
milp_solver.py        # Exact MILP solver (PuLP / CBC)
aco_solver.py         # ACO metaheuristic with time reservation table
visualization.py      # matplotlib + networkx visualisations
api/
  main.py             # FastAPI application
  models.py           # Pydantic request/response schemas
examples/
  small_example.py    # 2x3 grid, 2 drones — MILP + ACO
  medium_example.py   # 20 nodes, 4 drones — ACO
  compare_solvers.py  # Scaling benchmark
tests/
  test_milp.py
  test_aco.py
  test_constraints.py
  test_api.py
docs/
  model.typ           # Mathematical model (Typst)
output/               # Generated figures (PNG)
```

## Mathematical model

See [docs/model.typ](docs/model.typ) for the full MILP formulation.

**Objective**: minimise makespan $C_{\max}$ (time until last drone returns to base).

**Key constraints**:
- Every search node covered exactly once
- Flow conservation per drone
- MTZ subtour elimination
- Battery budget
- Temporal consistency (arrival times)
- Anti-collision (no two drones at the same node simultaneously)
# ACO_drone_swarm
