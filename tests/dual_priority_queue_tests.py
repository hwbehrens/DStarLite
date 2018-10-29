# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez

# Provides basic testing functionality for the DualPriorityQueue class

import sys
sys.path.append("/Users/hwbehren/JetBrains/PycharmProjects/DStarLite")

from src import dual_priority_queue as dpq

pq = dpq.DualPriorityQueue()

# testing initialization
assert pq.pop() is None
assert pq.size() is 0

# testing basic insertion
pq.push("key1", 5, 5)
assert pq.min_state() == (5, 1)
pq.push("key2", 5, 6)
assert pq.min_state() == (5, 2)
assert pq.size() == 2

# testing deletion
pq.delete_key("key_missing")
assert pq.size() == 2

# testing deletion and re-insertion of an existing key
pq.push("key1", 6, 6)
assert pq.size() == 2
assert pq.min_state() == (5, 1)

# testing straightforward deletion
pq.delete_key("key2")
assert pq.size() == 1
assert pq.min_state() == (6, 1)

# testing tie-breaking
pq.push("key3", 6, 2)
pq.push("key4", 7, 0)
assert pq.min_state() == (6, 2)
assert pq.size() == 3

# testing PQ exhaustion
assert pq.pop() == ("key3", 6, 2)
assert pq.min_state() == (6, 1)
assert pq.size() == 2
assert pq.pop() == ("key1", 6, 6)
assert pq.min_state() == (7, 1)
assert pq.size() == 1
assert pq.pop() == ("key4", 7, 0)
assert pq.min_state() == (None, 0)
assert pq.size() == 0

# all done
print("Testing complete!")
