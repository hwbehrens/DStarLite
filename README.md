# DStarLite
An experimental Python implementation based on the D* Lite Paper

The first 4 commands are example of how to run each of the 4 search strategies in the Pacman domain. 
The last 2 commands are examples of how the runtime and memory usage were tracked. 
Please run these commands from the pacman_domain directory.

Naive Replanning AStar:
python pacman.py -l tinyMaze -p SearchAgent -a fn=nrastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5

LPA*:
python pacman.py -l tinyMaze -p SearchAgent -a fn=lpastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5

DStarLite:
python pacman.py -l tinyMaze -p SearchAgent -a fn=dstarlite,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5

AStar:
python pacman.py -l tinyMaze -p SearchAgent -a fn=astar,prob=PositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5

Runtime measurement command:
time python pacman.py -l bigMaze -p SearchAgent -a fn=nrastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic  --frameTime 0 -z .5

Memory measurement command:
valgrind python pacman.py -l bigMaze -p SearchAgent -a fn=nrastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5
