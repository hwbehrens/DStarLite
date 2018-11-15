# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez

# Provides basic testing functionality for the DualPriorityQueue class

import sys
import math

sys.path.append("/Users/hwbehren/JetBrains/PycharmProjects/DStarLite")

from src import d_star_lite as dsl

# test map 1 (no internal walls)
#   ####
#   #S.#    map A
#   # G#
#   ####

map_a = dsl.DStarLite(start_coord=(0, 0), goal_coord=(1, 1), resolution=(2, 2))
step1 = map_a.take_step()
step2 = map_a.take_step()
assert map_a.extract_path() == [(0, 0), (1, 0), (1, 1)]

# test 2 (further, no walls)
#   #####
#   #S..#   map B
#   #  .#
#   #  G#
#   #####

map_b = dsl.DStarLite(start_coord=(0, 0), goal_coord=(2, 2), resolution=(3, 3))
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
map_c = dsl.DStarLite(start_coord=(0, 0), goal_coord=(0, 0), resolution=(10, 10))
assert map_c.extract_path() == [(0, 0)]

# test 6: unreachable case
#   #####
#   #S#G#
#   #####
map_d = dsl.DStarLite(start_coord=(0, 0), goal_coord=(2, 0), resolution=(3, 1))
map_d.make_wall_at((1, 0))
assert map_d.extract_path() is None

# all done
print("Testing complete!")
