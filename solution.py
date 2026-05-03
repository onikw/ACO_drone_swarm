"""Solution dataclass and serialisation for drone mission planning."""
from __future__ import annotations

import dataclasses
from typing import Optional


@dataclasses.dataclass
class Solution:
    """Complete mission solution for a multi-drone search problem.

    Attributes:
        routes: drone_id -> ordered list of node IDs starting and ending at base.
        schedule: drone_id -> list of (node, arrive_time, depart_time) tuples.
        makespan: mission completion time (when last drone returns to base).
        total_flight_time: sum of all travel times across all drones.
        battery_usage: drone_id -> total time consumed (travel + search).
        solver: name of the solver that produced this solution.
        solve_time: wall-clock seconds spent solving.
    """

    routes: dict[int, list[int]]
    schedule: dict[int, list[tuple[int, float, float]]]
    makespan: float
    total_flight_time: float
    battery_usage: dict[int, float]
    solver: str = "unknown"
    solve_time: float = 0.0

    # ------------------------------------------------------------------
    # Derived helpers
    # ------------------------------------------------------------------

    def drone_ids(self) -> list[int]:
        return list(self.routes.keys())

    def visited_nodes(self) -> set[int]:
        """All non-base nodes that appear in any route."""
        result: set[int] = set()
        for route in self.routes.values():
            result.update(route)
        return result

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "routes": {str(k): v for k, v in self.routes.items()},
            "schedule": {
                str(k): [(node, arr, dep) for node, arr, dep in v]
                for k, v in self.schedule.items()
            },
            "makespan": self.makespan,
            "total_flight_time": self.total_flight_time,
            "battery_usage": {str(k): v for k, v in self.battery_usage.items()},
            "solver": self.solver,
            "solve_time": self.solve_time,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Solution":
        routes = {int(k): v for k, v in d["routes"].items()}
        schedule = {
            int(k): [(node, arr, dep) for node, arr, dep in v]
            for k, v in d["schedule"].items()
        }
        battery_usage = {int(k): v for k, v in d["battery_usage"].items()}
        return cls(
            routes=routes,
            schedule=schedule,
            makespan=d["makespan"],
            total_flight_time=d["total_flight_time"],
            battery_usage=battery_usage,
            solver=d.get("solver", "unknown"),
            solve_time=d.get("solve_time", 0.0),
        )

    def __repr__(self) -> str:
        return (
            f"Solution(solver={self.solver!r}, makespan={self.makespan:.2f}, "
            f"drones={len(self.routes)}, solve_time={self.solve_time:.1f}s)"
        )
