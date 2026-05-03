"""Shared fixtures for pytest."""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from graph import SearchGraph


@pytest.fixture
def small_graph() -> SearchGraph:
    """Complete graph K5 (base=0, search nodes 1-4) with uniform weights.

    Every pair of nodes is directly connected, so any partition of search nodes
    between drones is topologically feasible.  Fixed weights make MILP optimal
    deterministic: makespan = 2+10+2+10+2 = 26 min for a 2-node sub-route.
    """
    g = SearchGraph(base_node=0)
    for v in range(5):
        g.add_node(v, search_time=10.0 if v > 0 else 0.0)
    for i in range(5):
        for j in range(i + 1, 5):
            g.add_edge(i, j, travel_time=2.0)
    return g


@pytest.fixture
def small_battery() -> list[float]:
    return [60.0, 60.0]


@pytest.fixture
def n_drones() -> int:
    return 2
