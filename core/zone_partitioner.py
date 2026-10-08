"""
Phase 8: Zone Partitioner

Divides the warehouse grid into Z spatial zones and provides:
  - Zone assignment for any (x, y) cell
  - Zone centroids (used by HierarchicalCoordinator for task routing)
  - Robot-to-zone assignment at startup (fixed for the run)
  - Sorted nearest-zone list for a given task location (for handoff fallback)

Partitioning strategy: simple quadrant/rectangular split — no external
dependencies (no scipy/sklearn needed). For Z=4 this gives a 2x2 grid of
zones; for Z=2 it splits horizontally. Scales to any Z by tiling in column-
major order (fill column by column).

This module is intentionally stateless and lightweight — just spatial math.
Nothing in this file touches the RWARE env, the auction layer, or the
pathfinder. It is imported by zone_auction_layer, zone_fault_layer, and
hierarchical_coordinator.
"""

import math
from typing import List, Tuple, Dict


class ZonePartitioner:
    """
    Rectangular spatial partitioner for a warehouse grid.

    Parameters
    ----------
    grid_width  : int  — number of columns in the grid (x-axis)
    grid_height : int  — number of rows in the grid (y-axis)
    n_zones     : int  — number of zones to create (default 4)

    Zone layout (column-major, left-to-right then top-to-bottom):
        For n_zones=4, grid split into 2 columns × 2 rows:
            Zone 0 | Zone 2
            -------+-------
            Zone 1 | Zone 3
    """

    def __init__(self, grid_width: int, grid_height: int, n_zones: int = 4):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.n_zones = n_zones

        # Determine grid of zone columns and rows
        # e.g. n_zones=4 -> n_cols=2, n_rows=2
        #      n_zones=5 -> n_cols=3, n_rows=2 (one col has fewer rows)
        self.n_cols = math.ceil(math.sqrt(n_zones))
        self.n_rows = math.ceil(n_zones / self.n_cols)

        # Width/height of each zone cell (last zone in each axis may be smaller)
        self.zone_col_width = grid_width / self.n_cols
        self.zone_row_height = grid_height / self.n_rows

        # Precompute centroids for all zones
        self._centroids: List[Tuple[float, float]] = []
        for z in range(n_zones):
            col = z // self.n_rows
            row = z % self.n_rows
            cx = (col + 0.5) * self.zone_col_width
            cy = (row + 0.5) * self.zone_row_height
            self._centroids.append((cx, cy))

    # ------------------------------------------------------------------
    # Core spatial queries
    # ------------------------------------------------------------------

    def get_zone(self, x: int, y: int) -> int:
        """
        Return the zone ID that cell (x, y) belongs to.
        Clamps to valid range so boundary cells always get a valid zone.
        """
        col = min(int(x / self.zone_col_width), self.n_cols - 1)
        row = min(int(y / self.zone_row_height), self.n_rows - 1)
        zone_id = col * self.n_rows + row
        # Clamp in case n_zones is not a perfect rectangle
        return min(zone_id, self.n_zones - 1)

    def get_centroid(self, zone_id: int) -> Tuple[float, float]:
        """Return the (x, y) centroid of the given zone."""
        return self._centroids[zone_id]

    def nearest_zones_to_task(self, task_x: float, task_y: float) -> List[int]:
        """
        Return all zone IDs sorted by Manhattan distance from (task_x, task_y)
        to each zone's centroid. Used by HierarchicalCoordinator to route a
        task (and find fallback zones for cross-zone handoff).
        """
        dists = []
        for z in range(self.n_zones):
            cx, cy = self._centroids[z]
            d = abs(task_x - cx) + abs(task_y - cy)
            dists.append((d, z))
        dists.sort()
        return [z for _, z in dists]

    # ------------------------------------------------------------------
    # Robot assignment
    # ------------------------------------------------------------------

    def assign_robots_to_zones(self, agents) -> Dict[int, int]:
        """
        Assign each robot (by its current position) to the zone it physically
        starts in. Assignment is fixed at call-time — robots do not migrate
        between zones during a run.

        Parameters
        ----------
        agents : list of RWARE agent objects (must have .x and .y attributes)

        Returns
        -------
        dict mapping robot_index -> zone_id
        """
        assignment = {}
        for i, agent in enumerate(agents):
            assignment[i] = self.get_zone(agent.x, agent.y)
        return assignment

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def print_zone_map(self):
        """Print an ASCII map of zone assignments for visual inspection."""
        print(f"Zone map ({self.grid_width}w x {self.grid_height}h, "
              f"{self.n_zones} zones, {self.n_cols}col x {self.n_rows}row):")
        for y in range(self.grid_height):
            row_str = ""
            for x in range(self.grid_width):
                row_str += str(self.get_zone(x, y))
            print(f"  {row_str}")
        print("Centroids:")
        for z, (cx, cy) in enumerate(self._centroids):
            print(f"  Zone {z}: centroid=({cx:.1f}, {cy:.1f})")

    def report_robot_assignment(self, assignment: Dict[int, int]):
        """Print the robot-to-zone assignment."""
        from collections import defaultdict
        by_zone = defaultdict(list)
        for robot_id, zone_id in assignment.items():
            by_zone[zone_id].append(robot_id)
        print("Robot-to-zone assignment:")
        for z in range(self.n_zones):
            print(f"  Zone {z}: robots {by_zone[z]}")
