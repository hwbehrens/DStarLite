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
import collections
import math

import dual_priority_queue_hans as dpq

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
class LPAStar(object):
    def __init__(self, problem, heuristic_func=l1_dist):
        self.initialize(problem, heuristic_func)

    # ########  LPA* core functions  ########
    def initialize(self, problem, heuristic_func):
        # init the containers
        self._h = heuristic_func  # expects a lambda that can be called
        self._U = dpq.DualPriorityQueue()
        self._start = problem.getStartState()
        self._goal = problem.getGoalState()
        self._has_path = False
        self._best_path = None
        self._last_path = None
        self._pop_count = 0
        self._curr_loc = None

        # set up the map/grid
        x_res, y_res = problem.getDims()
        self._width = x_res
        self._height = y_res
        self._node_count = x_res * y_res
        self._vertex_costs = [[[float("inf"), float("inf")] for _ in range(y_res)] for _ in range(x_res)]
        self._is_wall = problem.getNaiveWalls()

        # init the start node
        self._set_weight_tuple(self._start, (Unchanged, 0))
        prim, sec = self.compute_keys(self._start)
        self._U.push(key=self._start, primary=prim, secondary=sec)

    def compute_keys(self, coord):
        heuristic = self._h(coord, self._goal)
        tup = self._get_weight_tuple(coord)
        baseline = min(tup)
        return baseline + heuristic, baseline

    def update_vertex(self, coord, exclusion_node=None):
        if exclusion_node is None:
            exclusion_node = self._start

        # update rhs (if not start node)
        if coord != exclusion_node:
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
            self._pop_count += 1
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
            goal_keys = self.compute_keys(self._goal)
            peek_keys = self._U.peek()[1:3]

        self._has_path = True

    # ########  internal helper functions ########
    def _tuple_lt(self, tup1, tup2):
        if not isinstance(tup1, collections.Iterable):
            raise ValueError("Left-side tuple is not iterable: {0}".format(tup1))

        if not isinstance(tup2, collections.Iterable):
            raise ValueError("Right-side tuple is not iterable: {0}".format(tup2))

        if len(tup1) == 2:
            t1_primary, t1_secondary = tup1
        elif len(tup1) == 3:
            t1_label, t1_primary, t1_secondary = tup1
        else:
            raise ValueError("Left-side tuple contains unexpected arity: {0}".format(tup1))

        if len(tup2) == 2:
            t2_primary, t2_secondary = tup2
        elif len(tup2) == 3:
            t2_label, t2_primary, t2_secondary = tup2
        else:
            raise ValueError("Right-side tuple contains unexpected arity: {0}".format(tup2))

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
        all_dirs = [(coord[0], coord[1] + 1),  # north
                    (coord[0] + 1, coord[1]),  # east
                    (coord[0], coord[1] - 1),  # south
                    (coord[0] - 1, coord[1])]  # west
        valid_dirs = []
        for each in all_dirs:
            if self._in_map(each):
                valid_dirs.append(each)
        return valid_dirs

    def _set_weight_tuple(self, coord, tup):
        x, y = coord
        if tup[0] is not Unchanged:
            self._vertex_costs[x][y][0] = tup[0]
        if tup[1] is not Unchanged:
            self._vertex_costs[x][y][1] = tup[1]

    def _get_weight_tuple(self, coord):
        x, y = coord
        return self._vertex_costs[x][y]

    # ########  external (pacman) helper functions  ########

    def make_wall_at(self, coord):
        x, y = coord

        if self._is_wall[x][y]:
            return

        # path might have changed!
        self._has_path = False
        self._last_path = self._best_path
        self._best_path = None

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        self._is_wall[x][y] = True
        self.update_vertex(coord)

    def extract_path(self, backward=True):
        if self._start == self._goal:
            return [self._start]  # trivial case

        self.compute_shortest_path()  # if no shortest path is yet available, generate one
        if self._best_path is None:
            # traverses the weights and returns a series of coordinates corresponding to the shortest path
            best_path = []

            if not backward:
                curr_pos = self._start  # go from start to goal (D*lite)
                target_pos = self._goal
            else:
                curr_pos = self._goal  # go from goal to start (LPA*)
                target_pos = self._start
            if self._get_weight_tuple(curr_pos)[0] == float("inf"):
                return None  # no path between start and goal

            while curr_pos != target_pos:
                best_path.append(curr_pos)
                curr_neighbors = self._get_neighbors(curr_pos)
                for i in range(len(curr_neighbors)):
                    curr_neighbors[i] = (curr_neighbors[i], self._get_weight_tuple(curr_neighbors[i])[1])
                curr_neighbors.sort(key=lambda tup: tup[1])
                curr_pos = curr_neighbors[0][0]

            best_path.append(curr_pos)  # add the goal to the path
            self._best_path = best_path
            if backward:
                self._best_path.reverse()
        return self._best_path

    def get_path_intersection_point(self):
        if self._last_path is None or self._best_path is None:
            return

        pos = None
        for index in range(min(len(self._best_path), len(self._last_path))):
            if self._best_path[index] == self._last_path[index]:
                pos = self._best_path[index]
            else:
                return pos

    def get_backtrack_path(self):
        backpath = []
        intersect = self.get_path_intersection_point()
        if intersect is None:
            return []  # no path to backtrack to
        #        import pdb; pdb.set_trace()
        rev_path = reversed(self._last_path)
        for point in rev_path:
            if point != intersect:
                backpath.append(point)
            else:
                break
        backpath.append(intersect)
        return backpath

    def getRoute(self, index_coord=None):
        """
        Example:
        path = ['A', 'B', 'C', 'D', 'E']
        bt_path = ['G', 'F', 'H', 'C']

        Return: ['G', 'F', 'H', 'C', 'D', 'E']
        """
        path = self.extract_path()
        bt_path = self.get_backtrack_path()
        if not bt_path:
            return path
        intersection_pt = bt_path[-1]
        slice_index = path.index(intersection_pt)
        ret_path = bt_path + path[slice_index + 1:]
        if index_coord:
            ret_path = ret_path[ret_path.index(index_coord) + 1:]
        return ret_path
