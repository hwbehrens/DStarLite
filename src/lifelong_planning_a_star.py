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
import math

# global variables
EDGE_WEIGHT = 1

# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default


# baseline distance metrics
def l1_dist(coord1, coord2):
    p1, p2, q1, q2 = coord1 + coord2
    return abs(p1 - q1) + abs(p2 - q2)


def l2_dist(coord1, coord2):
    p1, p2, q1, q2 = coord1 + coord2
    return math.sqrt(((q1 - p1) ** 2) + ((q2 - p2) ** 2))


class LPAStar:
    # ########  LPA* core functions  ########

    def __init__(self, resolution, start_coord, goal_coord, heuristic_func=l1_dist):
        # corresponds to "Initialize"

        # init the containers
        self._h = heuristic_func  # expects a lambda that can be called
        self._pq = dpq.DualPriorityQueue()
        self._start = start_coord
        self._goal = goal_coord
        self._has_path = False

        # set up the map/grid
        x_res, y_res = resolution
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertex_weights = [[[float("inf"), float("inf")] for x in range(x_res)] for x in range(y_res)]
        self._is_wall = [[False for x in range(x_res)] for x in range(y_res)]

        # init the start node
        self._set_tuple(start_coord, (None, 0))
        prim, sec = self.compute_keys(start_coord)
        self._pq.push(key=start_coord, primary=prim, secondary=sec)

    def compute_keys(self, coord):
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
                if self._is_wall[each[0]][each[1]]:
                    new_rhs = min(new_rhs, float("inf"))  # it's a wall - infinite cost
                else:
                    new_rhs = min(new_rhs, g + EDGE_WEIGHT)  # it's not a wall - cost is 1
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
        goal_tup = self._get_tuple(self._goal)
        goal_keys = self.compute_keys(self._goal)
        peek_keys = self._pq.peek()[1:3]  # slice the peek to get the priority tuple only
        while (goal_tup[0] != goal_tup[1]) or (self._tuple_lt(peek_keys, goal_keys)):
            u = self._pq.pop()
            vertex_weight_tuple = self._get_tuple(u[0])
            if vertex_weight_tuple[0] > vertex_weight_tuple[1]:
                self._set_tuple(u, (vertex_weight_tuple[1], None))  # g(s) = rhs(s)
            else:
                self._set_tuple(u, (float("inf"), None))  # g(s) = infinity
                self.update_vertex(u)  # update the vertex itself
            for s in self._get_neighbors(u):
                self.update_vertex(s)  # update the successor vertices, in either case

            # prep for next iteration
            goal_tup = self._get_tuple(self._goal)
        self._has_path = True

    # ########  internal helper functions ########
    def _tuple_lt(self, tup1, tup2):
        if len(tup1) == 2 and len(tup2) == 2:
            t1_primary, t1_secondary = tup1
            t2_primary, t2_secondary = tup2
        elif len(tup1) == 3 and len(tup2) == 3:
            t1_label, t1_primary, t1_secondary = tup1
            t2_label, t2_primary, t2_secondary = tup2
        else:
            raise ValueError("Cannot compare tuples with different (or non-standard) input arity")

        if t1_primary < t2_primary:
            return True
        elif t1_primary > t2_primary:
            return False
        else:
            return t1_secondary < t2_secondary

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
            self._vertex_weights[x][y][0] = tup[0]
        if tup[1] is not None:
            self._vertex_weights[x][y][0] = tup[1]

    def _get_tuple(self, coord):
        x, y = coord
        return self._vertex_weights[x][y]

    # ########  external (pacman) helper functions  ########

    def convert_to_wall(self, coord):
        x, y = coord
        self._is_wall[x][y] = True
        self._has_path = False  # path might have changed!

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        for each in self._get_neighbors(coord):
            # if it's already a wall, don't worry about updating it
            each_x, each_y = each
            if not self._is_wall[each_x][each_y]:
                self.update_vertex(each)

    def extract_path(self):
        if not self._has_path:
            self.compute_shortest_path()  # if no shortest path is yet available, generate one

        # traverses the weights and returns a series of coordinates corresponding to the shortest path
        best_path = []

        curr_pos = self._start
        while curr_pos != self._goal:
            curr_neighbors = self._get_neighbors(curr_pos)
            for i in range(len(curr_neighbors)):
                # get the weights as well
                curr_neighbors[i] = (curr_neighbors[i], self._get_tuple(curr_neighbors[i]))
            print(curr_neighbors)  # ******

        return best_path
