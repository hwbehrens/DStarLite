# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez
#
# This implementation of LPA* based on the following paper:
# "Improved fast replanning for robot navigation in unknown terrain"
# https://doi.org/10.1109/ROBOT.2002.1013481


# imports
from src import dual_priority_queue as dpq

# global variables
EDGE_WEIGHT = 1

# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default


class LPAStar:

    # ########  LPA* core functions  ########

    def __init__(self, resolution, start_coord, goal_coord, heuristic_func, edge_list):
        # corresponds to "Initialize"

        # init the containers
        self._h = heuristic_func
        self._pq = dpq.DualPriorityQueue
        self._start = start_coord
        self._goal = goal_coord

        # set up the map/grid
        x_res, y_res = resolution
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertices = [[[float("inf"), float("inf")] for x in range(x_res)] for x in range(y_res)]
        self._inaccessible = [[False for x in range(x_res)] for x in range(y_res)]

        # init the start node
        self._set_tuple(start_coord, (None, 0))
        prim, sec = self.compute_keys(start_coord)
        self._pq.push(key=start_coord, primary=prim, secondary=sec)

    def compute_keys(self, coord):
        x, y = coord
        heuristic = self._h(coord, self._goal)
        tup = self._get_tuple(coord)
        baseline = min(tup)
        return baseline + heuristic, baseline

    def update_vertex(self, coord):
        # update rhs (if not start node)
        if coord != self._start:
            new_rhs = float("inf")
            for each in self._get_neighbors(coord):
                g, _ = self._get_tuple(each)
                new_rhs = min(new_rhs, g + EDGE_WEIGHT)
            self._set_tuple(coord, (None, new_rhs))

        # remove from PQ
        self._pq.delete_key(coord)

        # re-insert if locally inconsistent
        g, rhs = self._get_tuple(coord)
        if g != rhs:
            prim, sec = self.compute_keys(coord)
            self._pq.push(key=coord, primary=prim, secondary=sec)

    def compute_shortest_path(self):
        # implicitly assumes start to goal
        # @@@@@ pick up here
        pass

    # ########  internal helper functions ########

    def _in_map(self, coord):
        x, y = coord
        return 0 <= x < self._width and 0 <= y < self._height

    def _get_neighbors(self, coord):
        all_dirs = [(coord[0], coord[1] - 1), (coord[0] + 1, coord[1]), (coord[0], coord[1] + 1),
                    (coord[0] - 1, coord[1] - 1)]
        valid_dirs = []
        for each in all_dirs:
            if self._in_map(each):
                valid_dirs.append(each)
        return valid_dirs

    def _set_tuple(self, coord, tup):
        x, y = coord
        if tup[0] is not None:
            self._vertices[x][y][0] = tup[0]
        if tup[1] is not None:
            self._vertices[x][y][0] = tup[1]

    def _get_tuple(self, coord):
        x, y = coord
        return self._vertices[x][y]

    # ########  external (pacman) helper functions  ########

    def convert_to_wall(self, coord):
        x, y = coord
        self._inaccessible[x][y] = True

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        for each in self._get_neighbors(coord):
            # if it's already a wall, don't worry about updating it
            each_x, each_y = each
            if not self._inaccessible[each_x][each_y]:
                self.update_vertex(each)
