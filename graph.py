"""SearchGraph model for drone mission planning."""
import logging
from typing import Optional
import networkx as nx

logger = logging.getLogger(__name__)


class SearchGraph:
    """Directed search graph where each undirected edge is stored as two directed edges.

    Nodes carry a `search_time` attribute (minutes).
    Edges carry a `travel_time` attribute (minutes).
    One distinguished node is `base_node` (drone home base).
    """

    def __init__(self, base_node: int = 0) -> None:
        self._graph: nx.DiGraph = nx.DiGraph()
        self.base_node: int = base_node

    # ------------------------------------------------------------------
    # Mutation
    # ------------------------------------------------------------------

    def add_node(self, v: int, search_time: float) -> None:
        """Add a node with its search time (in minutes)."""
        self._graph.add_node(v, search_time=search_time)

    def add_edge(self, i: int, j: int, travel_time: float) -> None:
        """Add an undirected edge as two directed edges with the given travel time."""
        self._graph.add_edge(i, j, travel_time=travel_time)
        self._graph.add_edge(j, i, travel_time=travel_time)

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def nodes(self) -> list[int]:
        """All node IDs."""
        return list(self._graph.nodes())

    def search_nodes(self) -> list[int]:
        """All nodes except the base."""
        return [v for v in self._graph.nodes() if v != self.base_node]

    def edges(self):
        """All directed edges with data."""
        return self._graph.edges(data=True)

    def neighbors(self, v: int) -> list[int]:
        """Direct successors of v."""
        return list(self._graph.successors(v))

    def travel_time(self, i: int, j: int) -> float:
        """Travel time on the directed edge (i, j)."""
        return self._graph[i][j]["travel_time"]

    def search_time(self, v: int) -> float:
        """Search time at node v."""
        return self._graph.nodes[v]["search_time"]

    def has_edge(self, i: int, j: int) -> bool:
        return self._graph.has_edge(i, j)

    def n_nodes(self) -> int:
        return self._graph.number_of_nodes()

    def shortest_path(self, i: int, j: int) -> tuple[list[int], float]:
        """Dijkstra shortest path by travel_time. Returns (path, length)."""
        try:
            path = nx.dijkstra_path(self._graph, i, j, weight="travel_time")
            length = nx.dijkstra_path_length(self._graph, i, j, weight="travel_time")
            return path, length
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return [], float("inf")

    def shortest_path_length(self, i: int, j: int) -> float:
        """Shortest travel time between two nodes."""
        try:
            return nx.dijkstra_path_length(self._graph, i, j, weight="travel_time")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return float("inf")

    def get_layout(self, seed: int = 42) -> dict[int, tuple[float, float]]:
        """Spring-layout positions for visualization."""
        return nx.spring_layout(self._graph, seed=seed)

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        """Serialize the graph to a plain dict."""
        return {
            "base_node": self.base_node,
            "nodes": [
                {"id": v, "search_time": self._graph.nodes[v]["search_time"]}
                for v in self._graph.nodes()
            ],
            # Store only one direction per undirected edge to keep it compact.
            "edges": [
                {"i": i, "j": j, "travel_time": d["travel_time"]}
                for i, j, d in self._graph.edges(data=True)
                if i < j
            ],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SearchGraph":
        """Reconstruct a SearchGraph from a dict produced by to_dict()."""
        g = cls(base_node=d["base_node"])
        for node in d["nodes"]:
            g.add_node(node["id"], node["search_time"])
        for edge in d["edges"]:
            g.add_edge(edge["i"], edge["j"], edge["travel_time"])
        return g
