// Kompletny opis projektu: planowanie misji przeszukiwania obszaru przez wiele dronów
// Typst document — wersja 2.0 (aktualna)

#set document(title: "Planowanie misji przeszukiwania obszaru przez wiele dronów")
#set page(margin: 2cm, numbering: "1")
#set text(font: "New Computer Modern", size: 11pt)
#set heading(numbering: "1.")
#show heading.where(level: 1): it => {
  pagebreak(weak: true)
  it
}

// ============================================================
= Opis projektu

Projekt realizuje system planowania misji przeszukiwania obszaru przez flotę dronów.
Zadaniem jest wyznaczenie tras i harmonogramów tak, aby każdy węzeł (obszar do
przeszukania) był odwiedzony dokładnie przez jeden dron, przy jednoczesnej minimalizacji
czasu zakończenia całej misji (*makespan* — czas powrotu ostatniego drona do bazy).

System oferuje dwa solwery:

- *MILP* (Mixed-Integer Linear Programming) — solver dokładny oparty na bibliotece
  PuLP z backendem CBC; gwarantuje optimum globalne, praktyczny dla grafów do ~12 węzłów.

- *ACO* (Ant Colony Optimisation) — metaheurystyka przybliżona; skaluje się do dużych
  instancji, bez gwarancji optymalności, ale osiąga rozwiązania bliskie MILP-owi.

Projekt zawiera także walidator ograniczeń, REST API (FastAPI), wizualizacje
(matplotlib), testy jednostkowe i integracyjne (pytest) oraz przykłady uruchomienia.

// ============================================================
= Model grafu

== Definicja

Graf przeszukiwania $G = (V, E, v_0)$ jest *skierowanym* grafem ważonym, gdzie:

- $V$ — zbiór wszystkich węzłów (węzły przeszukiwania + baza),
- $E subset V times V$ — zbiór skierowanych krawędzi,
- $v_0 in V$ — baza dronów (węzeł startowy i końcowy),
- $s(v) >= 0$ — czas przeszukania węzła $v$ (minuty),
- $t(i,j) > 0$ — czas przelotu krawędzią $(i,j)$ (minuty).

Krawędzie są nieskierowane w sensie danych wejściowych (dodawana jest para $(i,j)$ i
$(j,i)$ z tym samym czasem), ale model wewnętrznie operuje na grafie skierowanym
($"nx.DiGraph"$), co umożliwia rozszerzenie na warianty asymetryczne.

Zbiór węzłów przeszukiwania: $V' = V \\ \{v_0\}$.

Najkrótsza ścieżka między węzłami obliczana jest algorytmem Dijkstry po krawędziach
z wagą $t(i,j)$.

== Implementacja

Klasa `SearchGraph` (plik `graph.py`) opakowuje `nx.DiGraph`. Kluczowe metody:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [`add_node(v, search_time)`], [Dodaje węzeł z czasem przeszukania],
  [`add_edge(i, j, travel_time)`], [Dodaje parę krawędzi skierowanych],
  [`shortest_path_length(i, j)`], [Dijkstra — najkrótsza odległość czasowa],
  [`shortest_path(i, j)`], [Dijkstra — ścieżka i długość],
  [`search_nodes()`], [Lista węzłów $V'$ (bez bazy)],
  [`to_dict()` / `from_dict()`], [Serializacja/deserializacja do/ze słownika],
  [`get_layout(seed)`], [Rozmieszczenie węzłów do wizualizacji (spring layout)],
)

== Generatory grafów

Plik `graph_generator.py` dostarcza trzy generatory z parametrem `seed` dla
powtarzalności:

=== Siatka prostokątna (`generate_grid_graph`)

Węzły o ID $v = r dot "cols" + c$ (row-major). Krawędzie łączą sąsiadów
poziomych i pionowych. Parametry: `rows`, `cols`, `search_time_range`,
`travel_time_range`, `base_node`, `seed`.

=== Graf losowy (`generate_random_graph`)

Gwarantuje spójność poprzez losowe drzewo rozpinające, następnie dodaje dodatkowe
krawędzie do osiągnięcia zadanej gęstości `density`. Parametry: `n`, `density`,
`search_time_range`, `travel_time_range`, `base_node`, `seed`.

=== Graf klastrowy (`generate_cluster_graph`)

Kilka gęstych podgrafów (kompletnych wewnątrz klastra) połączonych rzadkimi
krawędziami między klastrami. Parametry: `n_clusters`, `nodes_per_cluster`,
`search_time_range_intra`, `travel_time_range_inter`, `base_node`, `seed`.

// ============================================================
= Struktura rozwiązania

Klasa `Solution` (plik `solution.py`) jest dataclass Pythona przechowującą:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [`routes`], [$"drone\_id" -> [v_0, n_1, ..., n_k, v_0]$ — trasa drona],
  [`schedule`], [$"drone\_id" -> [(v, t_"arr", t_"dep"), ...]$ — harmonogram przeszukań],
  [`makespan`], [Czas zakończenia misji (minuty)],
  [`total_flight_time`], [Suma czasów przelotów wszystkich dronów],
  [`battery_usage`], [$"drone\_id" -> "czas"$ — zużycie baterii (przelot + przeszukanie)],
  [`solver`], [Nazwa solvera (`"MILP-CBC-transit"`, `"MILP-CBC-strict"`, `"ACO"`)],
  [`solve_time`], [Czas obliczeń (sekundy)],
)

Trasa (`route`) może zawierać *węzły tranzytowe* — węzły przez które dron przelatuje
bez przeszukiwania. Harmonogram (`schedule`) zawiera *wyłącznie* węzły faktycznie
przeszukane. To rozróżnienie jest kluczowe dla walidatora.

// ============================================================
= Model MILP

== Zbiory i parametry

#grid(
  columns: (auto, 1fr),
  gutter: 6pt,
  $V$, [zbiór wszystkich węzłów (z bazą $v_0$)],
  $V' = V \\ \{v_0\}$, [węzły przeszukiwania],
  $D$, [zbiór dronów $\{0, 1, ..., m-1\}$],
  $n = |V|$, [liczba węzłów],
  $s(v)$, [czas przeszukania węzła $v$],
  $t(i,j)$, [czas przelotu krawędzią $(i,j)$],
  $B_d$, [budżet baterii drona $d$],
  $M = 10000$, [stała Big-M dla ograniczeń wskaźnikowych],
)

== Zmienne decyzyjne

#grid(
  columns: (auto, 1fr),
  gutter: 6pt,
  $x_{i,j,d} in \{0,1\}$, [1 jeśli dron $d$ przelatuje krawędzią $(i arrow j)$],
  $y_{v,d} in \{0,1\}$, [1 jeśli dron $d$ przeszukuje węzeł $v$],
  $a_{v,d} >= 0$, [czas przybycia drona $d$ do węzła $v$],
  $u_{v,d} in [0,n]$, [zmienna MTZ eliminacji podcykli dla drona $d$ w węźle $v$],
  $z_{v,d_1,d_2} in \{0,1\}$, [1 jeśli $d_1$ odwiedza $v$ przed $d_2$],
  $C_"max" >= 0$, [makespan misji (cel minimalizacji)],
)

== Cel

$
  min quad C_"max"
$

== Ograniczenia

=== 1. Pokrycie — każdy węzeł przeszukany dokładnie raz

$
  sum_(d in D) y_{v,d} = 1, quad forall v in V'
$

=== 2–3. Zachowanie przepływu

==== 2a. Węzły przeszukiwania (tryb transit-allowed)

Dron może przelecieć przez węzeł bez przeszukania (węzeł tranzytowy).
Przepływ jest *zbilansowany*, a przeszukanie wymaga odwiedzenia:

$
  sum_(i in V) x_{i,v,d} = sum_(j in V) x_{v,j,d}, quad forall v in V', d in D
$
$
  sum_(i in V) x_{i,v,d} <= 1, quad forall v in V', d in D
$
$
  y_{v,d} <= sum_(i in V) x_{i,v,d}, quad forall v in V', d in D
$

==== 2b. Węzły przeszukiwania (tryb transit=search, historyczny)

W pierwotnym modelu każdy odwiedzony węzeł był jednocześnie przeszukiwany
($"flow\_in" = y_{v,d}$). Tryb ten jest dostępny jako `transit_allowed=False`
i wstępnie usuwał możliwość tranzytu, co w rzadkich grafach prowadziło do
niewykonalności przy wielu dronach.

==== 3. Baza

$
  sum_(j in V) x_{v_0,j,d} <= 1, quad
  sum_(j in V) x_{v_0,j,d} = sum_(i in V) x_{i,v_0,d}, quad forall d in D
$

=== 4. Spójność x–y (tylko transit=search)

Gdy `transit_allowed=False`, dodawane są ograniczenia:

$
  x_{i,j,d} <= y_{i,d} + [i = v_0], quad
  x_{i,j,d} <= y_{j,d} + [j = v_0], quad forall (i,j) in E, d in D
$

W trybie `transit_allowed=True` ograniczenia te są zbędne i pomijane.

=== 5. Eliminacja podcykli MTZ

$
  u_{i,d} - u_{j,d} + n dot x_{i,j,d} <= n - 1,
  quad forall i,j in V', d in D
$

=== 6. Budżet baterii

$
  sum_((i,j) in E) t(i,j) x_{i,j,d} + sum_(v in V') s(v) y_{v,d} <= B_d,
  quad forall d in D
$

=== 7–8. Spójność czasowa

==== 7. Węzeł przeszukiwania → węzeł przeszukiwania

W trybie transit-allowed czas przeszukania $s(i)$ wliczany jest tylko gdy dron
faktycznie przeszukuje węzeł $i$:

$
  a_{j,d} >= a_{i,d} + s(i) dot y_{i,d} + t(i,j) - M(1 - x_{i,j,d}),
  quad forall i,j in V', d in D
$

==== 8. Baza → węzeł przeszukiwania

$
  a_{j,d} >= t(v_0,j) - M(1 - x_{v_0,j,d}),
  quad forall j in V', d in D
$

=== 9. Antykolizja

Dwa drony nie mogą jednocześnie przeszukiwać tego samego węzła.
Zmienna binarna $z_{v,d_1,d_2}$ koduje kolejność:

$
  a_{v,d_1} + s(v) &<= a_{v,d_2}
    + M(1 - z_{v,d_1,d_2})
    + M(2 - y_{v,d_1} - y_{v,d_2}) \
  a_{v,d_2} + s(v) &<= a_{v,d_1}
    + M dot z_{v,d_1,d_2}
    + M(2 - y_{v,d_1} - y_{v,d_2})
$

dla każdej pary $(d_1, d_2)$ z $d_1 < d_2$ i każdego $v in V'$.

=== 10. Definicja makespanu

W trybie transit-allowed:

$
  C_"max" >= a_{v,d} + s(v) dot y_{v,d} + t(v, v_0) dot x_{v,v_0,d},
  quad forall v in V', (v,v_0) in E, d in D
$

== Implementacja MILP

Klasa `MILPSolver` (plik `milp_solver.py`) przyjmuje parametr `transit_allowed: bool = True`.
Solver PuLP/CBC wywoływany z limitem czasu `time_limit` (domyślnie 300 s) i
tolerancją optymalności `gapRel=0.0`.

Po znalezieniu rozwiązania metoda `_extract_solution` odtwarza:
- trasy przez przejście po aktywnych krawędziach ($x_{i,j,d} > 0.5$),
- harmonogramy tylko dla węzłów z $y_{v,d} > 0.5$,
- zużycie baterii jako sumę czasów przelotów aktywnych krawędzi + przeszukań.

Rekonstrukcja trasy z krawędzi: `_reconstruct_route` przechodzi po zbiorze
aktywnych krawędzi w kolejności (greedy walk), zaczynając i kończąc na bazie.

// ============================================================
= Algorytm ACO

== Przegląd

`ACOSolver` (plik `aco_solver.py`) buduje rozwiązania dla wszystkich dronów
jednocześnie. Kluczowa właściwość: każdy *mrówka* konstruuje kompletny plan misji
dla całej floty, nie pojedynczej trasy.

== Parametry

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt,
  [Parametr], [Domyślnie], [Opis],
  [`alpha`], [2.0], [Waga feromonu w selekcji probabilistycznej],
  [`beta`], [3.0], [Waga heurystyki (odwrotności kosztu)],
  [`rho`], [0.3], [Współczynnik wyparowania feromonu],
  [`n_ants`], [30], [Liczba mrówek na iterację],
  [`max_iter`], [200], [Maksymalna liczba iteracji],
  [`Q`], [1.0], [Współczynnik skalowania depozytu feromonu],
  [`local_search`], [True], [Czy stosować 2-opt po konstrukcji],
  [`seed`], [None], [Ziarno generatora losowego (powtarzalność)],
)

== Inicjalizacja feromonu

Globalny poziom feromonu $tau_0$ kalibrowany jest na podstawie heurystyki
najbliższego sąsiada:

$
  tau_0 = Q / (|V'| dot L_"nn")
$

gdzie $L_"nn"$ to długość trasy nearest-neighbour (jeden dron, sekwencyjny).

== Konstrukcja rozwiązania (jedna mrówka)

Algorytm działa w pętli, przypisując kolejne węzły $v in V'$ do dronów:

1. *Wybór drona*: dron z najwcześniejszym wolnym czasem (`earliest-available-first`).
2. *Lista kandydatów*: węzły nieodwiedzone, do których istnieje bezpośrednia
   krawędź z aktualnej pozycji drona, i które spełniają ograniczenie baterii:
   $"used" + t("pos", v) + s(v) + r(v) <= B_d$, gdzie $r(v)$ to koszt powrotu
   z $v$ do bazy (Dijkstra).
3. *Antykolizja*: czas przybycia korygowany przez `TimeReservationTable.earliest_free`.
4. *Selekcja probabilistyczna* (roulette-wheel):

$
  p(v | "pos") propto [tau("pos", v)]^alpha dot eta("pos", v)^beta,
  quad eta("pos", v) = 1 / max(t("pos", v) + s(v),; 10^{-6})
$

5. Dron wykonuje przeszukanie, rezerwuje okno w TRT, aktualizuje stan.
6. Gdy brak kandydatów — dron wraca do bazy i jest wykluczany z dalszej selekcji.

== Tabela rezerwacji czasu (TimeReservationTable)

Klasa `TimeReservationTable` przechowuje dla każdego węzła listę przedziałów
$[t_"start", t_"end")$ (half-open). Operacje:

- `check_conflict(node, t_s, t_e)` — czy $[t_s, t_e)$ nakłada się z rezerwacją,
- `reserve(node, t_s, t_e)` — dodaje rezerwację,
- `earliest_free(node, duration, not_before)` — najmniejszy $t >= "not\_before"$
  taki, że $[t, t+"duration")$ nie koliduje z żadną rezerwacją.

Algorytm `earliest_free` przechodzi przez posortowane okna i przesuwa $t$ za każde
okno, które pokrywa $[t, t+"duration")$.

== Lokalne przeszukiwanie 2-opt

Po konstrukcji każdej trasy (opcjonalnie) stosowany jest algorytm 2-opt minimalizujący
sumę czasów przelotów. Funkcja `_two_opt` używa `shortest_path_length` jako kosztu
segmentu, więc trasa nie musi korzystać wyłącznie z bezpośrednich krawędzi.
Tylko węzły wewnętrzne (`route[1:-1]`) są przestawiane; baza pozostaje nieruchoma.

== Aktualizacja feromonu (MMAS-style)

Po każdej iteracji:

1. *Wyparowanie*: $tau_(i,j) arrow.l max(tau_(i,j) dot (1-rho),; 10^{-10})$
2. *Depozyt* z najlepszej trasy iteracji i globalnej najlepszej:

$
  Delta tau = Q / C_"max"^*, quad tau_(i,j) += Delta tau
  quad "dla każdej krawędzi trasy"
$

Historia konwergencji (nierastąca) przechowywana jest w `convergence_history`.

// ============================================================
= Walidator ograniczeń

Funkcja `validate_solution(solution, graph, n_drones, battery_budgets)`
(plik `constraints.py`) zwraca listę komunikatów naruszeń (pusta lista = rozwiązanie
dopuszczalne).

Sprawdzane kategorie:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [Nr], [Opis sprawdzenia],
  [1], [*Pokrycie*: każdy węzeł $v in V'$ pojawia się w `schedule` dokładnie jednego drona],
  [2], [*Trasa: baza*: każda trasa zaczyna i kończy się w $v_0$],
  [3], [*Osiągalność*: dla każdej pary kolejnych węzłów w trasie $"shortest\_path\_length" < inf$],
  [4], [*Bateria*: zużycie (przeloty + przeszukania) $<= B_d + epsilon$],
  [5], [*Spójność harmonogramu*: węzły w schedule są podciągiem trasy; czasy przybycia\
        spełniają $a_v >= a_"prev" + s("prev") + d("prev", v)$; czas odjazdu $= a_v + s(v)$],
  [6], [*Antykolizja*: żadne dwa drony nie przeszukują tego samego węzła w nakładających\
        się oknach czasowych],
  [7], [*Makespan*: zadeklarowany makespan zgodny z obliczonym z harmonogramu],
)

Uwaga: pokrycie i bateria sprawdzane są na podstawie `schedule` (przeszukane węzły),
nie `route` (mogą zawierać węzły tranzytowe). Spójność harmonogramu używa
podciągu — węzły tranzytowe mogą pojawiać się między węzłami przeszukanymi w trasie.

// ============================================================
= REST API

Plik `api/main.py` implementuje serwer FastAPI z czterema endpointami.
Modele Pydantic zdefiniowane w `api/models.py`.

== Endpointy

=== `GET /health`

Zwraca `{"status": "ok"}`.

=== `POST /graph/generate`

Generuje graf testowy. Ciało żądania:
```json
{
  "type": "grid" | "random" | "cluster",
  "params": { ... }
}
```

Parametry `params` zależne od typu (patrz rozdz. 2.2). Zwraca `GraphModel`.

=== `POST /solve/milp`

Rozwiązuje problem MILP. Ciało żądania (`MILPRequest`):
```json
{
  "graph": { ... },
  "n_drones": 2,
  "battery_budgets": [60.0, 60.0],
  "time_limit": 120
}
```

Zwraca `SolutionResponse` z trasami, harmonogramem, makespannem i listą naruszeń.

=== `POST /solve/aco`

Rozwiązuje problem ACO. Ciało żądania (`ACORequest`) zawiera dodatkowo
parametry: `alpha`, `beta`, `rho`, `n_ants`, `max_iter`, `local_search`, `seed`.

=== `POST /validate`

Waliduje przekazane rozwiązanie. Ciało żądania (`ValidateRequest`):
```json
{
  "graph": { ... },
  "solution": { ... },
  "n_drones": 2,
  "battery_budgets": [60.0, 60.0]
}
```
Zwraca `{"feasible": bool, "violations": [...]}`.

// ============================================================
= Wizualizacje

Plik `visualization.py` dostarcza sześć funkcji matplotlib; każda przyjmuje
`save_path` (ścieżka PNG) i `show: bool`:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [`plot_graph(graph, ...)`], [Graf z węzłami i krawędziami (kolor = czas przeszukania)],
  [`plot_solution(graph, sol, ...)`], [Trasy dronów różnymi kolorami na tle grafu],
  [`plot_gantt(sol, ...)`], [Wykres Gantta: oś X = czas, oś Y = dron],
  [`plot_timeline(sol, ...)`], [Oś czasu z harmonogramem przeszukań],
  [`plot_convergence(history, ...)`], [Krzywa konwergencji ACO (makespan vs iteracja)],
  [`plot_comparison(milp_sol, aco_sol, ...)`], [Porównanie makespanów obu solwerów],
)

// ============================================================
= Testy

Testy napisane w pytest, uruchamiane z katalogu głównego projektu: `pytest tests/`.

== Wspólne fixtures (`tests/conftest.py`)

- `small_graph` — graf K5 (baza=0, węzły 1–4), czasy: $s(v)=10$ min, $t(i,j)=2$ min.
  Wybór pełnego grafu (zamiast siatki) gwarantuje, że dowolny podział węzłów między
  drony jest topologicznie wykonalny.
- `small_battery` — `[60.0, 60.0]`
- `n_drones` — `2`

== `test_milp.py` (6 testów)

Sprawdza: znalezienie rozwiązania, dopuszczalność (brak naruszeń), pokrycie,
start/koniec w bazie, respektowanie baterii, makespan > 0.

== `test_aco.py` (8 testów)

Sprawdza: znalezienie rozwiązania, dopuszczalność, pokrycie, monotoniczność
historii konwergencji, porównanie z MILP (ACO $<= 3 times$ MILP), oraz trzy
testy jednostkowe `TimeReservationTable`.

== `test_constraints.py` (7 testów)

Używa ręcznie skonstruowanego rozwiązania dla siatki 2×3. Testuje:
poprawne rozwiązanie bez naruszeń, brak pokrycia węzła, podwójne pokrycie,
przekroczenie baterii, kolizję, zły start trasy, zły koniec trasy.

== `test_api.py` (9 testów)

Testuje wszystkie endpointy REST API przy użyciu `httpx.AsyncClient`.

// ============================================================
= Przykłady uruchomienia

Plik `examples/small_example.py` ilustruje pełny przepływ:

1. Generacja siatki 2×3 (`seed=42`), $B_d = 120$ min, 2 drony.
2. Rozwiązanie MILP (tryb transit-allowed, limit 120 s).
3. Rozwiązanie ACO (20 mrówek, 100 iteracji, `seed=42`).
4. Walidacja obu rozwiązań.
5. Zapis wykresów do katalogu `output/`.
6. Wydruk makespanów i luki jakości.

Przykładowy wynik (siatka 2×3, `seed=42`):
- MILP: ~49.7 min
- ACO: ~54.0 min
- Luka: ~8.8%

// ============================================================
= Wymagania i zależności

Plik `requirements.txt`:

```
networkx       # graf i algorytmy grafowe (Dijkstra)
matplotlib     # wizualizacje
numpy          # obliczenia numeryczne
pulp           # modelowanie MILP (backend CBC)
fastapi        # REST API
uvicorn        # serwer ASGI
pydantic       # walidacja modeli API
pytest         # testy
httpx          # klient HTTP w testach API
```

Uruchomienie testów:
```bash
pytest tests/ -v
```

Uruchomienie API:
```bash
uvicorn api.main:app --reload
```

Uruchomienie przykładu:
```bash
python examples/small_example.py
```

// ============================================================
= Struktura plików projektu

```
.
├── graph.py              # SearchGraph — model grafu
├── graph_generator.py    # generatory: grid, random, cluster
├── solution.py           # Solution dataclass
├── constraints.py        # validate_solution()
├── milp_solver.py        # MILPSolver (PuLP/CBC)
├── aco_solver.py         # ACOSolver + TimeReservationTable
├── visualization.py      # 6 funkcji matplotlib
├── requirements.txt
├── api/
│   ├── main.py           # FastAPI app z 4 endpointami
│   └── models.py         # Pydantic modele żądań/odpowiedzi
├── tests/
│   ├── conftest.py       # fixtures: small_graph, small_battery, n_drones
│   ├── test_milp.py      # 6 testów MILP
│   ├── test_aco.py       # 8 testów ACO + TRT
│   ├── test_constraints.py # 7 testów walidatora
│   └── test_api.py       # 9 testów API
├── examples/
│   └── small_example.py  # przykład MILP vs ACO na siatce 2×3
├── docs/
│   └── model.typ         # niniejszy dokument (Typst)
└── output/               # katalog na wykresy PNG
```
