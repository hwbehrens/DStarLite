# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez
#
# This implementation of D* Lite based on the following paper:
# "Improved fast replanning for robot navigation in unknown terrain"
# https://doi.org/10.1109/ROBOT.2002.1013481

# imports
from src import lifelong_planning_a_star as lpa
from src import dual_priority_queue as dpq
import math

# global variables
EDGE_WEIGHT = 1
Unchanged = None  # alias to improve semantic readability

# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default


# noinspection PyAttributeOutsideInit
class DStarLite(lpa.LPAStar):
    def initialize(self, resolution, start_coord, goal_coord, heuristic_func=lpa.l1_dist):
        # init the containers
        self._h = heuristic_func  # expects a lambda that can be called
        self._U = dpq.DualPriorityQueue()
        self._km = 0  # used to offset the DPQ, to reduce rebalancing
        self._start = start_coord
        self._goal = goal_coord

        # set up the map/grid
        x_res, y_res = resolution
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertex_costs = [[[float("inf"), float("inf")] for _ in range(y_res)] for _ in range(x_res)]
        self._is_wall = [[False for _ in range(y_res)] for _ in range(x_res)]

        # init the start node
        self._set_weight_tuple(start_coord, (Unchanged, 0))
        prim, sec = self.compute_keys(start_coord)
        self._U.push(key=start_coord, primary=prim, secondary=sec)

    def compute_keys(self, coord):
        cost_tuple = self._get_weight_tuple(coord)
        secondary = min(cost_tuple)
        h_cost = self._h(self._start, coord)  # heuristic cost from start node to this node
        primary = secondary + h_cost + self._km
        return primary, secondary

    def update_vertex(self, coord):
        pass

    def compute_shortest_path(self):
        pass
