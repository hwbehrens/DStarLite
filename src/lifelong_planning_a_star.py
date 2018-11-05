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
Unchanged = None  # alias to improve semantic readability

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


# noinspection PyAttributeOutsideInit
class LPAStar:
    # ########  LPA* core functions  ########

    def __init__(self, resolution, start_coord, goal_coord, heuristic_func=l1_dist):
        self.initialize(resolution, start_coord, goal_coord, heuristic_func)

    def initialize(self, resolution, start_coord, goal_coord, heuristic_func):
        # init the containers
        self._h = heuristic_func  # expects a lambda that can be called
        self._U = dpq.DualPriorityQueue()
        self._start = start_coord
        self._goal = goal_coord
        self._has_path = False
        self._best_path = None

        # set up the map/grid
        x_res, y_res = resolution
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertex_weights = [[[float("inf"), float("inf")] for _ in range(y_res)] for _ in range(x_res)]
        self._is_wall = [[False for _ in range(y_res)] for _ in range(x_res)]

        # init the start node
        self._set_weight_tuple(start_coord, (Unchanged, 0))
        prim, sec = self.compute_keys(start_coord)
        self._U.push(key=start_coord, primary=prim, secondary=sec)

    def compute_keys(self, coord):
        heuristic = self._h(coord, self._goal)
        tup = self._get_weight_tuple(coord)
        baseline = min(tup)
        return baseline + heuristic, baseline

    def update_vertex(self, coord):
        # update rhs (if not start node)
        if coord != self._start:
            new_rhs = float("inf")  # if this node is a wall, the new rhs(s) is infinity
            if not self._is_wall[coord[0]][coord[1]]:
                neighbors = self._get_neighbors(coord)
                for each in neighbors:
                    # otherwise, it's [edge cost] more than the lowest neighboring g(s)
                    g, _ = self._get_weight_tuple(each)
                    new_rhs = min(new_rhs, g + EDGE_WEIGHT)
            self._set_weight_tuple(coord, (Unchanged, new_rhs))

        # remove from PQ
        self._U.delete_key(coord)

        # re-insert if locally underconsistent (inequality partially satisfied by upstream compute_shortest_path)
        g, rhs = self._get_weight_tuple(coord)
        if g != rhs:
            prim, sec = self.compute_keys(coord)
            self._U.push(key=coord, primary=prim, secondary=sec)

    def compute_shortest_path(self):
        if self._has_path:
            return  # don't try to re-compute an already-computed path, the PQ is empty

        # implicitly assumes start to goal
        goal_tup = self._get_weight_tuple(self._goal)
        goal_keys = self.compute_keys(self._goal)
        peek_keys = self._U.peek()[1:3]  # slice the peek to get the priority tuple only
        while (goal_tup[0] != goal_tup[1]) or (self._tuple_lt(peek_keys, goal_keys)):
            u = self._U.pop()[0]
            g_u, rhs_u = self._get_weight_tuple(u)  # pull these again; they may be different than when it was pushed
            if g_u > rhs_u:
                # locally overconsistent
                self._set_weight_tuple(u, (rhs_u, Unchanged))  # g(s) = rhs(s)
            else:
                self._set_weight_tuple(u, (float("inf"), Unchanged))  # g(s) = infinity
                self.update_vertex(u)  # update the vertex itself
            for s in self._get_neighbors(u):
                self.update_vertex(s)  # update the successor vertices, in either case

            # prep variables for next loop invariant test
            if self._U.size() == 0:
                break  # all done! the whole graph is consistent
            goal_tup = self._get_weight_tuple(self._goal)
            peek_keys = self._U.peek()[1:3]

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
            return True  # first primary wins
        elif t1_primary > t2_primary:
            return False  # second primary wins
        else:
            return t1_secondary < t2_secondary  # secondaries break tied primaries

    def _in_map(self, coord):
        x, y = coord
        return 0 <= x < self._width and 0 <= y < self._height

    def _get_neighbors(self, coord):
        all_dirs = [(coord[0], coord[1] - 1),  # north
                    (coord[0] + 1, coord[1]),  # east
                    (coord[0], coord[1] + 1),  # south
                    (coord[0] - 1, coord[1])]  # west
        valid_dirs = []
        for each in all_dirs:
            if self._in_map(each):
                valid_dirs.append(each)
        return valid_dirs

    def _set_weight_tuple(self, coord, tup):
        x, y = coord
        if tup[0] is not Unchanged:
            self._vertex_weights[x][y][0] = tup[0]
        if tup[1] is not Unchanged:
            self._vertex_weights[x][y][1] = tup[1]

    def _get_weight_tuple(self, coord):
        x, y = coord
        return self._vertex_weights[x][y]

    # ########  external (pacman) helper functions  ########

    def make_wall_at(self, coord):
        x, y = coord

        # path might have changed!
        self._has_path = False
        self._best_path = None

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        self._is_wall[x][y] = True
        self.update_vertex(coord)

    def extract_path(self):
        if self._start == self._goal:
            return [self._start]  # trivial case

        self.compute_shortest_path()  # if no shortest path is yet available, generate one
        if self._best_path is None:
            # traverses the weights and returns a series of coordinates corresponding to the shortest path
            best_path = []

            curr_pos = self._goal
            if self._get_weight_tuple(self._goal)[0] == float("inf"):
                return None  # no path exists to the goal node

            while curr_pos != self._start:
                best_path.append(curr_pos)
                curr_neighbors = self._get_neighbors(curr_pos)
                for i in range(len(curr_neighbors)):
                    curr_neighbors[i] = (curr_neighbors[i], self._get_weight_tuple(curr_neighbors[i])[1])
                curr_neighbors.sort(key=lambda tup: tup[1])
                curr_pos = curr_neighbors[0][0]

            best_path.append(curr_pos)  # add the goal to the path
            self._best_path = best_path
            self._best_path.reverse()
        return self._best_path
