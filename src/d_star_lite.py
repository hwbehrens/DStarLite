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
from src import dual_priority_queue as dpq
from src import lifelong_planning_a_star as lpa

# global variables
EDGE_WEIGHT = 1
Unchanged = None  # alias to improve semantic readability


# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default


# noinspection PyAttributeOutsideInit
class DStarLite(lpa.LPAStar):
    # ########  D* Lite core functions  ########
    def initialize(self, resolution, start_coord, goal_coord, heuristic_func=lpa.l1_dist):
        # init the containers
        self._h = heuristic_func  # expects a lambda that can be called
        self._U = dpq.DualPriorityQueue()
        self._km = 0  # used to offset the DPQ, to reduce rebalancing
        self._start = start_coord
        self._last = start_coord
        self._goal = goal_coord
        self._path = []
        self._changed_edges = set()

        # set up the map/grid
        x_res, y_res = resolution
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertex_costs = [[[float("inf"), float("inf")] for _ in range(y_res)] for _ in range(x_res)]
        self._is_wall = [[False for _ in range(y_res)] for _ in range(x_res)]

        # init the goal node (works backward)
        self._set_weight_tuple(goal_coord, (Unchanged, 0))
        prim, sec = self.compute_keys(goal_coord)
        self._U.push(key=goal_coord, primary=prim, secondary=sec)

        # find the path at least once, so we can take a step if needed
        self.compute_shortest_path()

    def compute_keys(self, coord):
        h_cost = self._h(self._start, coord)  # heuristic cost from start node to this node
        cost_tuple = self._get_weight_tuple(coord)
        secondary = min(cost_tuple)
        primary = secondary + h_cost + self._km
        return primary, secondary

    # we do not define update_vertex, as it can safely inherit from the superclass
    # note that directionality is inverted, but since our mobility graph is undirected, this is okay

    def compute_shortest_path(self):
        g_start, rhs_start = self._get_weight_tuple(self._start)
        while (self._U.size() > 0 and self._tuple_lt(self._U.peek(), self.compute_keys(self._start))) or \
                g_start != rhs_start:
            k_old = self._U.peek()
            u = self._U.pop()[0]
            g_u, rhs_u = self._get_weight_tuple(u)
            if self._tuple_lt(k_old, self.compute_keys(u)):
                prim, sec = self.compute_keys(u)
                self._U.push(u, prim, sec)
            elif g_u > rhs_u:
                self._set_weight_tuple(u, (rhs_u, Unchanged))  # g(u) = rhs(u)
                for each in self._get_neighbors(u):
                    self.update_vertex(each, self._goal)
            else:
                self._set_weight_tuple(u, (float("inf"), Unchanged))  # g(u) = inf
                for each in self._get_neighbors(u):
                    self.update_vertex(each, self._goal)
                self.update_vertex(u, self._goal)

            g_start, rhs_start = self._get_weight_tuple(self._start)  # update for next iteration

    # ########  external (pacman) helper functions  ########
    def make_wall_at(self, coord):
        x, y = coord

        start_neighbors = self._get_neighbors(self._start)
        if coord not in start_neighbors:
            raise ValueError("A wall cannot be discovered at a non-adjacent location; this breaks D* Lite.")

        self._is_wall[x][y] = True
        for each in self._get_neighbors(coord):
            self._changed_edges.add(each)  # add all wall-adjacent edges to the queue to be update_vertex'd

    def take_step(self):
        if self._start == self._goal:
            return self._start  # we're already at the goal; no need to move

        g_start, rhs_start = self._get_weight_tuple(self._start)
        if g_start == float("inf"):
            return self._start  # no path exists; no need to move

        # move according to our best guess
        argmin = (None, float("inf"))
        for each in self._get_neighbors(self._start):
            weight = EDGE_WEIGHT + self._get_weight_tuple(each)[0]
            if weight < argmin[1]:
                argmin = (each, weight)
        self._path.append(self._start)
        self._start = argmin[0]

        # check for edge updates
        if len(self._changed_edges) > 0:
            self._km += self._h(self._last, self._start)
            self._last = self._start
            for each in self._changed_edges:
                self.update_vertex(each, self._goal)
            self._changed_edges = set()  # we've updated all the edges
            self.compute_shortest_path()  # find the new shortest path

        return self._start

    def extract_path(self):
        return self._path
