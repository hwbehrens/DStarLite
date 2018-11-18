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
#   #S #    map A
#   #.G#
#   ####

map_a = dsl.DStarLite(start_coord=(0, 0), goal_coord=(1, 1), resolution=(2, 2))
map_a.take_step()
map_a.take_step()
path = map_a.get_route()
assert path == [(0, 0), (1, 0), (1, 1)]

# test 2 (further, no walls)
#   #####
#   #S  #   map B
#   #.  #
#   #..G#
#   #####

map_b = dsl.DStarLite(start_coord=(0, 0), goal_coord=(2, 2), resolution=(3, 3))
assert map_b.extract_path() == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]  # expected path
map_b.take_step()
map_b.take_step()
map_b.take_step()
map_b.take_step()
map_b.take_step()
assert map_b.get_route() == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]  # actual path should match expected

# test 3: discover a blocking wall, revise plan
#   #####
#   #S  #
#   #.. #
#   ##.G#
#   #####
map_b = dsl.DStarLite(start_coord=(0, 0), goal_coord=(2, 2), resolution=(3, 3))
map_b.take_step()
map_b.make_wall_at((2, 0))  # i suddenly see a wall in front of me!
assert map_b.extract_path() == [(1, 0), (1, 1), (2, 1), (2, 2)]  # expected path
map_b.take_step()
map_b.take_step()
map_b.take_step()
assert map_b.get_route() == [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2)]  # actual (overall) path

# test 4: discover ANOTHER blocking wall, revise plan
#   #####
#   #S..#
#   #.#.#
#   ## G#
#   #####
map_b = dsl.DStarLite(start_coord=(0, 0), goal_coord=(2, 2), resolution=(3, 3))
map_b.take_step()
map_b.make_wall_at((1, 1))  # dead end, now what?
map_b.make_wall_at((2, 0))
path = map_b.extract_path()
assert path == [(1, 0), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
map_b.take_step()
map_b.take_step()
map_b.take_step()
map_b.take_step()
map_b.take_step()
map_b.take_step()
assert map_b.get_route() == [(0, 0), (1, 0), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

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
map_d.take_step()  # can't move, should do nothing
try:
    map_d.make_wall_at((2, 0))
except ValueError as e:
    assert "non-adjacent" in str(e)  # cannot insert walls at non-adjacent locations

# all done
print("Testing complete!")
