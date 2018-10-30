# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez
#
# This implementation of LPA* based on the following paper:
# "Improved fast replanning for robot navigation in unknown terrain"
# https://doi.org/10.1109/ROBOT.2002.1013481
#


class DualPriorityQueue:
    def __init__(self):
        self._size = 0
        self._ledger = dict()  # maps keys to priority tuples
        self._priority = dict()  # maps primary priorities to keys
        self._min_value = None
        self._min_count = 0

    def size(self):
        return self._size

    def min_state(self):
        return self._min_value, self._min_count

    def push(self, key, primary, secondary):
        if key in self._ledger:
            self.delete_key(key)  # if a key is pushed twice, update it to the new value (?)

        self._ledger[key] = (primary, secondary)
        if primary not in self._priority:
            self._priority[primary] = []
        self._priority[primary].append(key)
        self._size += 1
        if self._min_value is None or primary < self._min_value:
            self._min_value = primary
            self._min_count = 1
        elif primary == self._min_value:
            self._min_count += 1

    def delete_key(self, key):
        if key in self._ledger:
            # clean up the data first
            old_primary, old_secondary = self._ledger[key]
            self._size -= 1
            del self._ledger[key]
            self._priority[old_primary].remove(key)
            if len(self._priority[old_primary]) == 0:
                del self._priority[old_primary]  # remove the dangling empty priority

            # can be expensive [O(n)] to recompute the min, so be smart about it
            if old_primary == self._min_value:
                self._min_count -= 1
                if self._min_count == 0:
                    self._compute_min()  # only compute if we've exhausted the current min-priority tier

    def _compute_min(self):
        if self.size() == 0:
            self._min_count = 0
            self._min_value = None
        else:
            priority_list = list(self._priority.keys())
            if len(priority_list) == 0:
                print(priority_list)
            self._min_value = min(priority_list)
            self._min_count = priority_list.count(self._min_value)

    def peek(self):
        if self.size() <= 0:
            return None  # nothing to pop

        # find and break the ties
        candidate_keys = self._priority[self._min_value]
        result = []
        for each in candidate_keys:
            primary, secondary = self._ledger[each]
            result.append((each, primary, secondary))
        result.sort(key=lambda x: x[2])  # sort the list by the secondary priority
        result = result[0]

        return result

    def pop(self):
        result = self.peek()
        if result is not None:
            self.delete_key(result[0])  # remove and clean up
        return result


