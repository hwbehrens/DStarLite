# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Actions
import lifelong_planning_a_star_hans as lpa
import d_star_lite_hans as dsl
from game import Directions

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from sets import Set
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    # Each element in the fringe stores this tuple:
    # (state, actions to get to the state).
    fringe = util.Stack()
    fringe.push((startState, []))
    visitedStates = Set()
    while not fringe.isEmpty():
        currState, actionsToCurrState = fringe.pop()
        if currState in visitedStates:
            # This check is necessary, because duplicate states can be in a
            # fringe.  It is possible that a duplicate state, which is now
            # visited, could have been in the fringe.
            continue
        if problem.isGoalState(currState):
            return actionsToCurrState
        visitedStates.add(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
            if successor not in visitedStates:
                fringe.push((successor, actionsToCurrState + [action]))
    # Goal not found, so no action.
    return []

    """
    from sets import Set
    startState = problem.getStartState()
    # Each element in the fringe stores this tuple:
    # (state, actions to get to the state).
    fringe = util.Stack()
    fringe.push((startState, []))
    #itemsInFringe = Set([startState])
    visitedStates = []
    while not fringe.isEmpty():
        currState, actionsToCurrState = fringe.pop()
        if problem.isGoalState(currState):
            return actionsToCurrState
     #   itemsInFringe.remove(currState)
        visitedStates.append(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
    #        if problem.isGoalState(successor):
    #            return actionsToCurrState + [action]  --> not allowed by autograder
            if successor not in visitedStates: #and successor not in \
    #                itemsInFringe:  --> not allowed by autograder
                fringe.push((successor, actionsToCurrState + [action]))
      #          itemsInFringe.add(successor)
    # Goal not found, so no action.
    return []
    """

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from sets import Set
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    # Each element in the fringe stores this tuple:
    # (state, actions to get to the state).
    fringe = util.Queue()
    fringe.push((startState, []))
    itemsInFringe = Set([startState])
    visitedStates = Set()
    while not fringe.isEmpty():
        currState, actionsToCurrState = fringe.pop()
        itemsInFringe.remove(currState)
        if problem.isGoalState(currState):
            return actionsToCurrState
        visitedStates.add(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
            if successor not in visitedStates and successor not in itemsInFringe:
                fringe.push((successor, actionsToCurrState + [action]))
                itemsInFringe.add(successor)
    # Goal not found, so no action.
    return []

    """
    from sets import Set
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    fringe = util.Queue()
    fringe.push((startState, []))
    visitedStates = Set()
    while not fringe.isEmpty():
        currState, actionsToCurrState = fringe.pop()
        if problem.isGoalState(currState):
            return actionsToCurrState
        visitedStates.add(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
            if problem.isGoalState(successor):
                return actionsToCurrState + [action]
            if successor not in visitedStates and successor not in fringe.list:
                fringe.push((successor, actionsToCurrState + [action]))
    return []
    """

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from sets import Set
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    # Each element in the fringe stores the state and the cost to reach it.
    fringe = util.PriorityQueue()
    fringe.push(startState, 0)
    # Each pair in itemsInFringe stores a state and the list of actions
    # required to reach it.  States are added in itemsInFringe when they are
    # added to the fringe.  The states are removed from itemsInFringe when
    # they get removed from the fringe.
    itemsInFringe = {startState: []}
    visitedStates = Set()
    while not fringe.isEmpty():
        currState = fringe.pop()
        actionsToCurrState = itemsInFringe[currState]
        del itemsInFringe[currState]
        costOfActionsToCurrState = problem.getCostOfActions(actionsToCurrState)
        if problem.isGoalState(currState):
            return actionsToCurrState
        visitedStates.add(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
            newCostToSuccessor = costOfActionsToCurrState + stepCost
            newActionsToSuccessor = actionsToCurrState + [action]
            if successor not in visitedStates:
                fringe.update(successor, newCostToSuccessor)
                if successor in itemsInFringe and \
                        problem.getCostOfActions(itemsInFringe[successor]) <= \
                        newCostToSuccessor:
                    # If successor is already in itemsInFringe, only update the
                    # cost if the current cost is greater than the new cost.
                    continue
                itemsInFringe[successor] = newActionsToSuccessor
    # Goal not found, so no action.
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from sets import Set
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    # Each element in the fringe stores the state and the cost to reach it.
    fringe = util.PriorityQueue()
    fringe.push(startState, 0 + heuristic(startState, problem))
    # Each pair in itemsInFringe stores a state and the list of actions
    # required to reach it.  States are added in itemsInFringe when they are
    # added to the fringe.  The states are removed from itemsInFringe when
    # they get removed from the fringe.
    itemsInFringe = {startState: []}
    visitedStates = Set()
    while not fringe.isEmpty():
        currState = fringe.pop()
        actionsToCurrState = itemsInFringe[currState]
        del itemsInFringe[currState]
        costOfActionsToCurrState = problem.getCostOfActions(actionsToCurrState)
        if problem.isGoalState(currState):
            return actionsToCurrState
        visitedStates.add(currState)
        for successor, action, stepCost in problem.getSuccessors(currState):
            heuristicCostToSuccessor = heuristic(successor, problem)
            newCostToSuccessor = costOfActionsToCurrState + stepCost + \
                    heuristicCostToSuccessor
            newActionsToSuccessor = actionsToCurrState + [action]
            if successor not in visitedStates:
                fringe.update(successor, newCostToSuccessor)
                if successor in itemsInFringe and \
                        problem.getCostOfActions(itemsInFringe[successor]) + \
                        heuristicCostToSuccessor <= newCostToSuccessor:
                    # If successor is already in itemsInFringe, only update the
                    # cost if the current cost is greater than the new cost.
                    continue
                itemsInFringe[successor] = newActionsToSuccessor
    # Goal not found, so no action.
    return []

def replanningAStarSearch(problem, heuristic):
    """
    Applies AStarSearch in the scenario where the agent only knows the goal
    state and does not know the location of the walls.  When the agent finds out
    a location of a wall from its successor states, it will need to restart the
    AStarSearch.

    We can initally test the implementation of this algorithm with tinyMaze grid
    using this command:
    python pacman.py -l tinyMaze -p SearchAgent -a fn=rastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic

    Then we can verify that the algorithm works with other grids, using the
    layouts from the layouts/ directory.
    """
    startState = problem.getStartState()
    x, y = startState[0], startState[1]
    pathSoFar = []
    while not problem.isGoalState((x, y)):
        actions = aStarSearch(problem, heuristic)
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if problem.isWall(nextx, nexty):
                problem.setNaiveWalls(nextx, nexty)
                problem.setStartState(x, y)
                break
            else:
                x = nextx
                y = nexty
                pathSoFar.append(action)
    return pathSoFar

def LPAStarSearch(problem):
    def getDirection(coord, nextCoord):
        curr_x = coord[0]
        next_x = nextCoord[0]
        curr_y = coord[1]
        next_y = nextCoord[1]
        if curr_x - next_x < 0:
            return Directions.EAST
        elif curr_x - next_x > 0:
            return Directions.WEST
        elif curr_y - next_y < 0:
            return Directions.NORTH
        else:
            return Directions.SOUTH

    startState = problem.getStartState()
    x, y = startState[0], startState[1]
    lpastar_obj = lpa.LPAStar(problem)
    path = []
    while not problem.isGoalState((x, y)):
        path.append((x, y))
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if problem.isWall(nextx, nexty):
                lpastar_obj.make_wall_at((nextx, nexty))
        coords = lpastar_obj.getRoute((x, y))
#        print 'coords:', coords
#        print 'path:', path
        # XXX: possibility that coords is an empty list
#        if x == 25 and y == 14:
#            import pdb; pdb.set_trace()
        try:
            nextCoord = coords[coords.index((x, y)) + 1]
        except:
            nextCoord = coords[0]
        x, y = nextCoord
    path.append((x, y))
    directions = []
    for index in range(len(path) - 1):
        coord = path[index]
        nextCoord = path[index + 1]
        direction = getDirection(coord, nextCoord)
        directions.append(direction)
    print 'path:', path
    return directions

def DStarLiteSearch(problem):
    def getDirection(coord, nextCoord):
        curr_x = coord[0]
        next_x = nextCoord[0]
        curr_y = coord[1]
        next_y = nextCoord[1]
        if curr_x - next_x < 0:
            return Directions.EAST
        elif curr_x - next_x > 0:
            return Directions.WEST
        elif curr_y - next_y < 0:
            return Directions.NORTH
        else:
            return Directions.SOUTH

    startState = problem.getStartState()
    x, y = startState[0], startState[1]
    dstarlite_obj = dsl.DStarLite(problem)
    while (x, y) != problem.getGoalState():
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if problem.isWall(nextx, nexty):
                dstarlite_obj.make_wall_at((nextx, nexty))
        x, y = dstarlite_obj.take_step()
    path = dstarlite_obj.get_route()
    directions = []
    for index in range(len(path) - 1):
        coord = path[index]
        nextCoord = path[index + 1]
        direction = getDirection(coord, nextCoord)
        directions.append(direction)
    problem._expanded = dstarlite_obj._pop_count
    return directions

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
rastar = replanningAStarSearch
lpastar = LPAStarSearch
dstarlite = DStarLiteSearch
