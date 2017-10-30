# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""
import util
from util import *

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # drmax$ python pacman.py -l mediumMaze -p SearchAgent
    # All of your search functions need to return a list of actions that will lead the agent from the start to the goal.
    # create a path variable
    path = [];
    # create a variable to store the visited nodes
    visited = [];
    # call the function for the path
    path = zzz(problem.getStartState(),path, problem, visited,"Initial")

    return path

def zzz(thisNode, path, problem, visited, directon):

    # adding the node to the path.
    path.append(directon)

    if problem.isGoalState(thisNode):
        # print "FOUND IT"
        # remove the first "Initial" direction
        path.pop(0)
        return path

    # "visit" the node
    visited.append(thisNode)
    # get the neighbouting nodes to which we are allowed to go to
    adjacentNodes = problem.getSuccessors(thisNode);
    # for every one of those nodes, run DFS
    for node in adjacentNodes:
        if node[0] not in visited:
            callResult = zzz(node[0], path, problem, visited, node[1])
            if callResult != None:
                return callResult

    # remove the node since it has noting valuable
    path.pop()
    return None


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "To launch: python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs"
    "*** YOUR CODE HERE ***"
    # declaring path and queue
    path = []
    queue = []
    visited = []

    # next 5 lines are here coz the first node (startState()) is only a pair of coordinates
    adjacentNodes = problem.getSuccessors(problem.getStartState())
    for n in adjacentNodes:
        if n not in visited:
            newNode = Node(n[0],n[1], '')
            queue.append(newNode)

    # while the queue is not empty
    while len(queue) != 0:
        #get the next item from a queue
        thisNode = queue.pop(0)
        # visit the node
        visited.append(thisNode.loc)
        # end if the node is the goal
        if problem.isGoalState(thisNode.loc):
            return thisNode.absPath

        #find the neighbors of the current node
        adjacentNodes = problem.getSuccessors(thisNode.loc)

        for n in adjacentNodes:
            if n[0] not in visited:
                newNode = Node(n[0],n[1], thisNode.absPath)
                queue.append(newNode)
    return None

# This class is needed to keep track of all of the paths for every single node
class Node:
    def __init__(self, coor, relPath, parentAbsFromCenter):
        self.loc = coor
        self.relPth = relPath
        self.absPath = list(parentAbsFromCenter)
        self.absPath.append(relPath)


def uniformCostSearch(problem):
    "Search the node of total cost first. "
    "Sort by the lowest cost"
    """
    python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
    python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
    python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
    """
    "*** YOUR CODE HERE ***"
    # initialize the priority queue
    priorityQueue = PriorityQueue()
    # make the start state coordinates into a node
    startNode = ucsNode(problem.getStartState(),"","",0)
    # insert the Start state into the priority queue
    priorityQueue.push(startNode,0)

    visited = []

    while True and not priorityQueue.isEmpty():
        # get the current node
        currentNode = priorityQueue.pop()

        if problem.isGoalState(currentNode.loc):
            print "found a path"
            path = currentNode.absPath
            path.pop(0)
            print path
            return path

        visited.append(currentNode.loc)

        adjacentNodes = problem.getSuccessors(currentNode.loc)

        for node in adjacentNodes:
            if node[0] not in visited:
                #create a new node
                newNode = ucsNode(node[0],node[1],currentNode.absPath,currentNode.costSoFar + node[2])
                # add the node to the priority queue
                priorityQueue.push(newNode, currentNode.costSoFar + node[2])

    return None


class ucsNode:
    def __init__(self, coor, relPath, parentAbsFromCenter, costSoFar):
        self.loc = coor
        self.relPth = relPath
        self.absPath = list(parentAbsFromCenter)
        self.absPath.append(relPath)
        self.costSoFar = costSoFar



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
