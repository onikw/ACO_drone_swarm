"""Generators for test SearchGraph instances."""
import logging
import random
from typing import Optional

import networkx as nx

from graph import SearchGraph

logger = logging.getLogger(__name__)


def _rng(seed: Optional[int]) -> random.Random:
    return random.Random(seed)


def generate_grid_graph(
    rows: int,
    cols: int,
    search_time_range: tuple[float, float] = (5.0, 30.0),
    travel_time_range: tuple[float, float] = (1.0, 5.0),
    base_node: int = 0,
    seed: Optional[int] = None,
) -> SearchGraph:
    """Rectangular grid graph — typical for area search missions.

    Node IDs are row-major: node at (r, c) has id r*cols + c.
    Edges connect horizontal and vertical neighbours.
    """
    rng = _rng(seed)
    g = SearchGraph(base_node=base_node)

    for r in range(rows):
        for c in range(cols):
            v = r * cols + c
            g.add_node(v, search_time=rng.uniform(*search_time_range))

    for r in range(rows):
        for c in range(cols):
            v = r * cols + c
            if c + 1 < cols:
                u = r * cols + (c + 1)
                g.add_edge(v, u, rng.uniform(*travel_time_range))
            if r + 1 < rows:
                u = (r + 1) * cols + c
                g.add_edge(v, u, rng.uniform(*travel_time_range))

    logger.debug("Generated grid graph %dx%d (%d nodes)", rows, cols, rows * cols)
    return g


def generate_random_graph(
    n: int,
    density: float = 0.4,
    search_time_range: tuple[float, float] = (5.0, 30.0),
    travel_time_range: tuple[float, float] = (1.0, 5.0),
    base_node: int = 0,
    seed: Optional[int] = None,
) -> SearchGraph:
    """Random connected graph with approximately n*density*(n-1)/2 edges.

    Connectivity is guaranteed by first building a random spanning tree, then
    adding extra edges until the target density is reached.
    """
    rng = _rng(seed)
    g = SearchGraph(base_node=base_node)

    for v in range(n):
        g.add_node(v, search_time=rng.uniform(*search_time_range))

    # Spanning tree (random permutation → chain)
    perm = list(range(n))
    rng.shuffle(perm)
    for k in range(n - 1):
        i, j = perm[k], perm[k + 1]
        g.add_edge(i, j, rng.uniform(*travel_time_range))

    # Extra edges up to density
    max_edges = n * (n - 1) // 2
    target = max(n - 1, int(density * max_edges))
    existing = {(min(i, j), max(i, j)) for i, j, _ in g.edges()}

    candidates = [
        (i, j) for i in range(n) for j in range(i + 1, n) if (i, j) not in existing
    ]
    rng.shuffle(candidates)
    for i, j in candidates[: target - (n - 1)]:
        g.add_edge(i, j, rng.uniform(*travel_time_range))

    logger.debug("Generated random graph n=%d density=%.2f", n, density)
    return g


def generate_cluster_graph(
    n_clusters: int,
    nodes_per_cluster: int,
    search_time_range: tuple[float, float] = (5.0, 30.0),
    travel_time_range_intra: tuple[float, float] = (1.0, 3.0),
    travel_time_range_inter: tuple[float, float] = (3.0, 8.0),
    base_node: int = 0,
    seed: Optional[int] = None,
) -> SearchGraph:
    """Cluster graph: several dense groups connected by sparse inter-cluster edges.

    Intra-cluster edges use shorter travel times; inter-cluster bridges use longer ones.
    """
    rng = _rng(seed)
    n = n_clusters * nodes_per_cluster
    g = SearchGraph(base_node=base_node)

    for v in range(n):
        g.add_node(v, search_time=rng.uniform(*search_time_range))

    # Dense intra-cluster edges (complete sub-graphs)
    for c in range(n_clusters):
        start = c * nodes_per_cluster
        end = start + nodes_per_cluster
        for i in range(start, end):
            for j in range(i + 1, end):
                g.add_edge(i, j, rng.uniform(*travel_time_range_intra))

    # One bridge edge between consecutive clusters to guarantee connectivity
    for c in range(n_clusters - 1):
        i = c * nodes_per_cluster + rng.randint(0, nodes_per_cluster - 1)
        j = (c + 1) * nodes_per_cluster + rng.randint(0, nodes_per_cluster - 1)
        g.add_edge(i, j, rng.uniform(*travel_time_range_inter))

    logger.debug(
        "Generated cluster graph %d clusters x %d nodes", n_clusters, nodes_per_cluster
    )
    return g
