# Multi-Drone Area Search Scheduler

System planowania misji przeszukiwania obszaru przez flotę dronów. Każdy węzeł grafu (obszar do zbadania) musi zostać odwiedzony przez dokładnie jednego drona; cel to minimalizacja makespanu — czasu powrotu ostatniego drona do bazy.

Dwa solvery: **MILP** (dokładny, PuLP/CBC, praktyczny do ~12 węzłów) i **ACO** (metaheurystyka, skalowalna do 20+ węzłów).

---

## Instalacja

```bash
uv sync          # zalecane
# lub
pip install -r requirements.txt
```

---

## Szybki start (CLI)

```bash
# Domyślna konfiguracja z config.json (siatka 2×3, 2 drony, MILP + ACO)
uv run python3 main.py

# Tylko ACO, graf losowy, 3 drony
uv run python3 main.py --solver aco --graph-type random --random-n 12 --n-drones 3 --battery 90

# Własny graf z pliku JSON
uv run python3 main.py --graph-file moj_graf.json --solver aco

# Wykresy interaktywnie (bez zapisu)
uv run python3 main.py --no-save --show
```

---

## Plik konfiguracyjny

Wszystkie parametry można ustawić w `config.json`. CLI zawsze nadpisuje config.

```json
{
  "graph": {
    "type": "grid",          // "grid" | "random" | "cluster" | "file"
    "file": { "path": "moj_graf.json" },
    "grid":    { "rows": 2, "cols": 3, "base_node": 0, "seed": 42,
                 "search_time_range": [5.0, 30.0], "travel_time_range": [1.0, 5.0] },
    "random":  { "n": 10, "density": 0.5, "base_node": 0, "seed": 42,
                 "search_time_range": [5.0, 30.0], "travel_time_range": [1.0, 5.0] },
    "cluster": { "n_clusters": 3, "nodes_per_cluster": 4, "base_node": 0, "seed": 42,
                 "search_time_range": [5.0, 30.0],
                 "travel_time_range_intra": [1.0, 3.0], "travel_time_range_inter": [3.0, 8.0] }
  },
  "mission": {
    "n_drones": 2,
    "battery_budgets": [120.0, 120.0]
  },
  "solver": "both",          // "milp" | "aco" | "both"
  "aco": {
    "alpha": 2.0,            // waga feromonu
    "beta": 3.0,             // waga heurystyki
    "rho": 0.3,              // współczynnik wyparowania
    "n_ants": 30,
    "max_iter": 200,
    "Q": 1.0,
    "local_search": true,    // 2-opt + inter-route relocate
    "seed": 42,
    "progress_every": 10     // pasek postępu co N iteracji (0 = wyłączony)
  },
  "milp": {
    "time_limit": 300,
    "transit_allowed": true  // false = tryb ścisły (każdy odwiedzony węzeł = przeszukany)
  },
  "output": {
    "directory": "output",
    "save_plots": true,
    "show_plots": false,
    "log_level": "INFO"      // "DEBUG" | "INFO" | "WARNING" | "ERROR"
  }
}
```

---

## Własny graf (format JSON)

Utwórz plik JSON zgodny z poniższym schematem i przekaż przez `--graph-file` lub `config.json`:

```json
{
  "base_node": 0,
  "nodes": [
    {"id": 0, "search_time": 0.0},
    {"id": 1, "search_time": 15.0},
    {"id": 2, "search_time": 10.0},
    {"id": 3, "search_time": 20.0}
  ],
  "edges": [
    {"i": 0, "j": 1, "travel_time": 3.0},
    {"i": 0, "j": 2, "travel_time": 5.0},
    {"i": 1, "j": 3, "travel_time": 4.0},
    {"i": 2, "j": 3, "travel_time": 2.0}
  ]
}
```

- `base_node` — ID węzła-bazy (start i koniec każdej trasy)
- `search_time` bazy powinien być `0.0`
- Krawędzie są nieskierowane — wpisz każdą parę tylko raz
- `travel_time` i `search_time` w minutach
- Graf musi być spójny (każdy węzeł osiągalny z bazy)

Eksport wygenerowanego grafu do pliku:

```python
from graph_generator import generate_random_graph
import json

g = generate_random_graph(n=10, seed=42)
with open("moj_graf.json", "w") as f:
    json.dump(g.to_dict(), f, indent=2)
```

---

## Opcje CLI (pełna lista)

```
python3 main.py [OPCJE]

Ogólne:
  --config FILE, -c FILE       plik konfiguracyjny (domyślnie: config.json)
  --solver {milp,aco,both}

Graf:
  --graph-type {grid,random,cluster,file}
  --graph-file FILE            ścieżka do pliku JSON z własnym grafem
  --graph-seed N
  --graph-base-node N
  --grid-rows, --grid-cols
  --random-n N, --random-density D
  --cluster-n-clusters N, --cluster-nodes-per-cluster N

Misja:
  --n-drones N
  --battery B [B ...]          jeden budżet → identyczny dla wszystkich dronów

ACO:
  --aco-alpha α, --aco-beta β, --aco-rho ρ
  --aco-n-ants N, --aco-max-iter N, --aco-Q Q
  --aco-no-local-search
  --aco-seed N
  --aco-progress-every N       pasek postępu co N iteracji (0 = wyłączony)

MILP:
  --milp-time-limit SEC
  --milp-no-transit            tryb ścisły (bez węzłów tranzytowych)

Wyjście:
  --output-dir DIR
  --no-save                    nie zapisuj plików PNG
  --show                       wyświetl wykresy interaktywnie
  --log-level {DEBUG,INFO,WARNING,ERROR}
```

---

## Wyniki działania

Program wyświetla w terminalu:

```
--- ACO solver ---
  [████████████░░░░░░░░░░░░░░░░]   60/200  best=49.67  iter=52.31  2s  vs NN: +8.1%

  Initial (NN) : makespan=54.02 min
    Drone 0: 0 → 1 → 2 → 5 → 0
    Drone 1: 0 → 3 → 4 → 0

  Final  (ACO) : Solution(solver='ACO', makespan=49.67, drones=2, solve_time=6.1s)
  Violations   : none
  Improvement over NN : +8.1%
  Best at iteration   : 12/200
```

Oraz zapisuje wykresy PNG do katalogu `output/`:

| Plik | Opis |
|---|---|
| `graph.png` | Wizualizacja grafu z wagami |
| `nn_initial_routes.png` | Trasy rozwiązania startowego (NN) |
| `aco_routes.png` | Trasy finalnego rozwiązania ACO |
| `aco_gantt.png` | Wykres Gantta (oś X = czas, oś Y = dron) |
| `aco_convergence.png` | Krzywa konwergencji (makespan vs iteracja) |
| `milp_routes.png` | Trasy MILP (jeśli uruchomiony) |
| `milp_gantt.png` | Wykres Gantta MILP |
| `comparison.png` | Porównanie MILP vs ACO |

---

## REST API

```bash
uv run uvicorn api.main:app --reload
# Swagger UI: http://localhost:8000/docs
```

| Metoda | Ścieżka | Opis |
|--------|---------|------|
| GET | `/health` | Health check |
| POST | `/graph/generate` | Generuj graf (`grid`, `random`, `cluster`) |
| POST | `/solve/milp` | Rozwiąż MILP/CBC (do ~12 węzłów) |
| POST | `/solve/aco` | Rozwiąż ACO (20+ węzłów) |
| POST | `/validate` | Zwaliduj gotowe rozwiązanie |

---

## Testy

```bash
uv run pytest tests/ -v
```

30 testów: MILP (6), ACO + TRT (8), walidator ograniczeń (7), REST API (9).

---

## Struktura projektu

```
main.py               # CLI entry point (argparse + config.json)
config.json           # Domyślna konfiguracja
graph.py              # SearchGraph (networkx DiGraph)
graph_generator.py    # Generatory: grid / random / cluster
solution.py           # Solution dataclass
constraints.py        # Walidator dopuszczalności (7 kategorii)
milp_solver.py        # Solver dokładny (PuLP / CBC)
aco_solver.py         # ACO z MMAS, TRT, 2-opt, inter-route relocate
visualization.py      # 6 funkcji matplotlib
api/
  main.py             # FastAPI (4 endpointy)
  models.py           # Pydantic schematy
tests/
  conftest.py
  test_milp.py
  test_aco.py
  test_constraints.py
  test_api.py
examples/
  small_example.py    # siatka 2×3, MILP + ACO
  medium_example.py   # 20 węzłów, ACO
  compare_solvers.py  # benchmark skalowalności
docs/
  model.typ           # Pełny model matematyczny (Typst)
output/               # Wygenerowane wykresy PNG
```

---

## Model matematyczny

Cel: minimalizacja makespanu $C_{\max}$ (czas powrotu ostatniego drona do bazy).

Ograniczenia:
- Każdy węzeł przeszukany dokładnie raz
- Każda trasa zaczyna i kończy się w bazie
- Eliminacja podcykli (MTZ)
- Budżet baterii per dron
- Spójność czasowa przybyć
- Antykolizja (żadne dwa drony jednocześnie w tym samym węźle)

Szczegółowy model MILP i opis ACO: [docs/model.typ](docs/model.typ)
