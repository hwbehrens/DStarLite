# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez

# Provides basic testing functionality for the DualPriorityQueue class

import math
import sys

sys.path.append("/Users/hwbehren/JetBrains/PycharmProjects/DStarLite")

from src import lifelong_planning_a_star as lpa

# test manhattan distance
assert lpa.l1_dist((5, 5), (5, 5)) == 0
assert lpa.l1_dist((0, 0), (0, 0)) == 0
assert lpa.l1_dist((0, 0), (5, 5)) == 10
assert lpa.l1_dist((5, 5), (0, 0)) == 10

# test euclidean distance
assert lpa.l2_dist((5, 5), (5, 5)) == 0
assert lpa.l2_dist((0, 0), (0, 0)) == 0
assert lpa.l2_dist((0, 0), (5, 5)) == math.sqrt(50)
assert lpa.l2_dist((5, 5), (0, 0)) == math.sqrt(50)

# test map 1 (no internal walls)
#   ####
#   #S.#    map A
#   # G#
#   ####

map_a = lpa.LPAStar(start_coord=(0, 0), goal_coord=(1, 1), resolution=(2, 2))
assert map_a.extract_path() == [(0, 0), (1, 0), (1, 1)]

# test 2 (further, no walls)
#   #####
#   #S..#   map B
#   #  .#
#   #  G#
#   #####

map_b = lpa.LPAStar(start_coord=(0, 0), goal_coord=(2, 2), resolution=(3, 3))
assert map_b.extract_path() == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]

# test 3: discover a blocking wall, revise plan
#   #####
#   #S.##
#   # ..#
#   #  G#
#   #####
map_b.make_wall_at((2, 0))
assert map_b.extract_path() == [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2)]

# test 3b: calling the same extraction twice uses the cached version from the last call
assert map_b.extract_path() == [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2)]

# test 3c: can't explicitly compute the path if it's been computed
map_b.compute_shortest_path()  # will crash if it tries to execute (improper error checking)

# test 4: discover ANOTHER blocking wall, revise plan
#   #####
#   #S ##
#   #.# #
#   #..G#
#   #####
map_b.make_wall_at((1, 1))
assert map_b.extract_path() == [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

# test 5: trivial case where goal == start
map_c = lpa.LPAStar(start_coord=(0, 0), goal_coord=(0, 0), resolution=(10, 10))
assert map_c.extract_path() == [(0, 0)]

# test 6: unreachable case
#   #####
#   #S#G#
#   #####
map_d = lpa.LPAStar(start_coord=(0, 0), goal_coord=(2, 0), resolution=(3, 1))
map_d.make_wall_at((1, 0))
assert map_d.extract_path() is None  # no path to find
assert map_d.get_backtrack_path() == []  # definitely nothing to backtrack to either

# test 7: tuple inequality testing
tup1, tup2 = ("a", 2, 1), ("b", 2, 2)
assert map_d._tuple_lt(tup1, tup2)
tup1, tup2 = (2, 2), (2, 1)
assert not (map_d._tuple_lt(tup1, tup2))
tup1 = (1)
try:
    res = map_d._tuple_lt(tup1, tup2)
except ValueError as e:
    assert "Left-side" in str(e)
try:
    res = map_d._tuple_lt(tup2, tup1)
except ValueError as e:
    assert "Right-side" in str(e)
tup1 = (1, 2, 3, 4, 5)
try:
    res = map_d._tuple_lt(tup1, tup2)
except ValueError as e:
    assert "Left-side" in str(e)
try:
    res = map_d._tuple_lt(tup2, tup1)
except ValueError as e:
    assert "Right-side" in str(e)

# test 8: surprise-wall test
#   ############  5 rows
#   #          # 10 cols
#   # ######## #
#   #S       #G#
#   # ######## #
#   #          #
#   ############
map_e = lpa.LPAStar(start_coord=(2, 0), goal_coord=(2, 9), resolution=(5, 10))

# make the corridor walls (we know about these at the start)
for y in range(1, 9):
    map_e.make_wall_at((1, y))
    map_e.make_wall_at((3, y))

# the naive, straight-across path
assert map_e.extract_path() == \
       [(2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (2, 8), (2, 9)]

# make the plug wall (zounds, so close! we're at (2,7), now what?)
map_e.make_wall_at((2, 8))

# go along the bottom row
assert map_e.extract_path() == \
       [(2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
        (4, 6), (4, 7), (4, 8), (4, 9), (3, 9), (2, 9)]

# how to backtrack to use the new path?
backpath = map_e.get_backtrack_path()

# all done
print("Testing complete!")
