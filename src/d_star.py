# Contributors:
# Hans Behrens
# Barath Gunari
# Rishabh Hatgadkar
# Nicholas Martinez
#
# This implementation of D* Lite based on the following paper:
# "Improved fast replanning for robot navigation in unknown terrain"
# https://doi.org/10.1109/ROBOT.2002.1013481
#
import enum


class Point:
    # this is a helper class that provides functionality used in the graph traversal

    class PointState(enum):
        # note that this uses the past-tense to get around the 'raise' keyword reservation
        new = 1
        opened = 2
        closed = 3
        raised = 4
        lowered = 5

    def __init__(self, init_cost=0, init_neighbors=None, init_state=PointState.new):
        # let the accessors handle error checking
        self.cost = None
        self.set_cost(init_cost)
        self.neighbors = None
        self.set_neighbors(init_neighbors)
        self.state = None
        self.set_state(init_state)

    # neighbor accessors
    def get_neighbors(self):
        return list(self.neighbors)

    def set_neighbors(self, new_neighbors):
        if new_neighbors is None:
            self.neighbors = list()  # handles the initialization case
        else:
            self.neighbors = list(new_neighbors)

    # cost accessors
    def get_cost(self):
        return float(self.cost)

    def set_cost(self, new_cost):
        self.cost = float(new_cost)

    # state accessors
    def get_state(self):
        return self.state

    def set_state(self, new_state):
        if isinstance(new_state, Point.PointState):
            self.state = new_state
        else:
            raise ValueError("Point set to illegal state: {0}".format(new_state))

    def is_state(self, state):
        return str(state) == self.get_state()


class DStar:
    def search(self):
        pass

    def expand(self, current_point):
        pass

    def is_raised(self):
        pass
