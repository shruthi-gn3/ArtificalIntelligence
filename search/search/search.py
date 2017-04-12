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

    visited = {}
    fringe = util.Stack()
    f_st = problem.getStartState()
    fr_item = []
    fr_item.append((f_st,'Start',0))
    fringe.push(fr_item)
    visited[f_st] = True
    final = None
    i = 0
    while not fringe.isEmpty():
        curr_state = fringe.pop()
        
        if problem.isGoalState(curr_state[-1][0]):
            final = curr_state
            break
        #test = curr_state[-1][0]
        visited[curr_state[-1][0]] = True
        expanded_state = []
        all_succ = problem.getSuccessors(curr_state[-1][0])
        # all_succ.reverse()
        for step in all_succ:
            if step[0] not in visited:
                #print str(i)+") "+str(step)
                ext = list(curr_state)
                ext.append(step)
                expanded_state.append(ext)
                #visited[step[0]] = True
       

        for item in expanded_state:
            fringe.push(item)
            
    path = []
    for item in final[1:]:
        path.append(item[1])
        #print item

    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = []
    succ_table = {}
    fringe = util.Queue()
    f_st = problem.getStartState()
    fr_item = []
    fr_item.append((f_st,'Start',0))
    fringe.push(fr_item)
    visited.append(f_st)
    final = None
    i = 0
    while not fringe.isEmpty():
        curr_state = fringe.pop()
        
        if problem.isGoalState(curr_state[-1][0]):
            final = curr_state
            
            break
        expanded_state = []
        all_succ = problem.getSuccessors(curr_state[-1][0])
        
        # all_succ.reverse()
        for step in all_succ:
            if step[0] not in visited:
                #print str(i)+") "+str(step)
                ext = list(curr_state)
                ext.append(step)
                expanded_state.append(ext)
                visited.append(step[0])
                #print visited
            
        for item in expanded_state:
            fringe.push(item)
            
    path = []
 
    for item in final[1:]:
        path.append(item[1])
        #print item
  

    return path

        
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = {}
    succ_table = {}
    fringe = util.PriorityQueue()
    f_st = problem.getStartState()
    fr_item = []
    fr_item.append((f_st,'Start',0))
    fringe.push(fr_item,0)
    #visited[f_st] = True
    final = None
    i = 0
    while not fringe.isEmpty():
        curr_state = fringe.pop()
      
        if problem.isGoalState(curr_state[-1][0]):
            final = curr_state
            break
        
        expanded_state = []
        if curr_state[-1][0] in succ_table:
            all_succ = succ_table[curr_state[-1][0]]
        else:
            all_succ = problem.getSuccessors(curr_state[-1][0])
            succ_table[curr_state[-1][0]] = all_succ
            #visited[curr_state[-1][0]] = True
        
        
        # all_succ.reverse()
        for step in all_succ:
            if step[0] not in [x for x,y,z in curr_state]:
                #print str(i)+") "+str(step)
                ext = list(curr_state)
                ext.append(step)
                expanded_state.append(ext)
                #visited[step[0][0]][step[0][1]] = True
       

        for item in expanded_state:
            cost = 0
            for i in item:
                cost = cost + i[2]
            fringe.update(item,cost)
            
    path = []
    for item in final[1:]:
        path.append(item[1])
        #print item
    #print path

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = []
    succ_table = {}
    fringe = util.PriorityQueue()
    f_st = problem.getStartState()
    fr_item = []
    fr_item.append((f_st,'Start',0))
    fringe.push(fr_item,0)
    final = None
    i = 0
    while not fringe.isEmpty():
        curr_state = fringe.pop()
      
        if problem.isGoalState(curr_state[-1][0]):
            final = curr_state
            break
        
        expanded_state = []
        if curr_state[-1][0] not in visited:
            visited.append(curr_state[-1][0])
            all_succ = problem.getSuccessors(curr_state[-1][0])
            for step in all_succ:
                if not step[0] in visited:
                    #print str(i)+") "+str(step)
                    ext = list(curr_state)
                    ext.append(step)
                    expanded_state.append(ext)
             
            for item in expanded_state:
                cost = problem.getCostOfActions([y for x,y,z in item[1:]]) + heuristic(item[-1][0],problem)
                fringe.push(item,cost)
            
    path = []
    for item in final[1:]:
        path.append(item[1])
        
    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
