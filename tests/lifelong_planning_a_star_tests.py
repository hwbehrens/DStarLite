# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez

# Provides basic testing functionality for the DualPriorityQueue class

import sys
import math

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
#   #S #
#   # G#
#   ####

map1 = lpa.LPAStar(start_coord=(0, 0), goal_coord=(1, 1), resolution=(2, 2))
assert (map1.extract_path() == [(0, 0), (0, 1), (1, 1)] or
        map1.extract_path() == [(0, 0), (1, 0), (1, 1)])

# all done
print("Testing complete!")
