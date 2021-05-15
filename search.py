#21100034 Waliullah Aitemad
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
	return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):

	
	current_state = problem.getStartState()

	visited = set()
	frontier = util.Stack()

	frontier.push((current_state,[],0))

	while not frontier.isEmpty():
		location, path, cost = frontier.pop()

		if problem.isGoalState(location):
			return path

		if location not in visited:
			visited.add(location)

			for s_location, s_path, s_cost in problem.getSuccessors(location):
				if s_location not in visited:
					frontier.push((s_location, path + [s_path], s_cost + cost))

	return []
				

# ________________________________________________________________


class _RecursiveDepthFirstSearch(object):
	"""
	=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
	above.
	Key Point: Remember in tutorial you were asked to expand the left-most child
	first for dfs and bfs for consistency. If you expanded the right-most
	first, dfs/bfs would be correct in principle but may not return the same
	path.

	=> Useful Hint: self.problem.getSuccessors(node) will return children of
	a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal
	is different from 'iterative' traversal, try reversing the sequence.

	"""

	def __init__(self, problem):
		" Do not change this. "
		# You'll save the actions that recursive dfs found in self.actions.
		self.actions = []
		# Use self.explored to keep track of explored nodes.
		self.explored = set()
		self.problem = problem

	def RecursiveDepthFirstSearchHelper(self, node):

		location, path = node, []

		if self.problem.isGoalState(location):
			return True

		if location not in self.explored:
			self.explored.add(location)

			for s_location, s_path, s_cost in self.problem.getSuccessors(location):
				if s_location not in self.explored:
					
					if self.RecursiveDepthFirstSearchHelper(s_location):

						self.actions.append(s_path)
						return True

		return False 
		



def RecursiveDepthFirstSearch(problem):
	" You need not change this function. "
	# All your code should be in member function 'RecursiveDepthFirstSearchHelper' of
	# class '_RecursiveDepthFirstSearch'."

	node = problem.getStartState()
	rdfs = _RecursiveDepthFirstSearch(problem)
	path_found = rdfs.RecursiveDepthFirstSearchHelper(node)
	if path_found:

		return list(
			reversed(rdfs.actions)
		)  # Actions your recursive calls return are in opposite order.


# ________________________________________________________________


def depthLimitedSearch(problem, limit=210):

	"""
	Search the deepest nodes in the search tree first as long as the
	nodes are not not deeper than 'limit'.

	For medium maze, pacman should find food for limit less than 130.
	If your solution needs 'limit' more than 130, it's bogus.
	Specifically, for:
	'python pacman.py -l mediumMaze -p SearchAgent -a fn=dls', and limit=130
	pacman should work normally.

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.
	Autograder cannot test this function.

	Hints: You may need to store additional information in your frontier(queue).

	"""

	"Start of Your Code"

	current_state = problem.getStartState()

	visited = set()
	frontier = util.Stack()

	frontier.push((current_state,[],0,0))

	while not frontier.isEmpty():
		location, path, cost, depth = frontier.pop()

		if problem.isGoalState(location):
			return path

		if location not in visited:
			visited.add(location)

			for s_location, s_path, s_cost in problem.getSuccessors(location):
				if s_location not in visited:
					if  depth<=limit:
						frontier.push((s_location, path + [s_path], s_cost + cost, depth+1))

	return []

	"End of Your Code"


# ________________________________________________________________


class _RecursiveDepthLimitedSearch(object):
	"""
	=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
	above.
	Key Point: Remember in tutorial you were asked to expand the left-most child
	first for dfs and bfs for consistency. If you expanded the right-most
	first, dfs/bfs would be correct in principle but may not return the same
	path.

	=> Useful Hint: self.problem.getSuccessors(node) will return children of
	a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal
	is different from 'iterative' traversal, try reversing the sequence.

	"""

	def __init__(self, problem):
		" Do not change this. "
		# You'll save the actions that recursive dfs found in self.actions.
		self.actions = []
		# Use self.explored to keep track of explored nodes.
		self.explored = set()
		self.problem = problem
		self.current_depth = 0
		self.depth_limit = 204  # For medium maze, You should find solution for depth_limit not more than 204.

	def RecursiveDepthLimitedSearchHelper(self, node):
		"""
		args: start node
		outputs: bool => True if path found else Fasle.
		"""

		"Start of Your Code"
		
		location, path = node, []
		++self.current_depth

		if self.problem.isGoalState(location):
			return True

		if location not in self.explored:
			self.explored.add(location)

			for s_location, s_path, s_cost in self.problem.getSuccessors(location):
				if s_location not in self.explored:
					if self.current_depth <= self.depth_limit:
						if self.RecursiveDepthLimitedSearchHelper(s_location):

							self.actions.append(s_path)
							--self.current_depth
							return True

		return False 
		"End of Your Code"


def RecursiveDepthLimitedSearch(problem):
	"You need not change this function. All your code in member function RecursiveDepthLimitedSearchHelper"
	node = problem.getStartState()
	rdfs = _RecursiveDepthLimitedSearch(problem)
	path_found = rdfs.RecursiveDepthLimitedSearchHelper(node)
	return list(
		reversed(rdfs.actions)
	)  # Actions your recursive calls return are in opposite order.


# ________________________________________________________________


def breadthFirstSearch(problem):
	"""Search the shallowest nodes in the search tree first."""

	current_state = problem.getStartState()

	visited = set()
	frontier = util.Queue()

	frontier.push((current_state,[],0))

	while not frontier.isEmpty():
		location, path, cost = frontier.pop()

		if problem.isGoalState(location):
			return path

		if location not in visited:
			visited.add(location)

			for s_location, s_path, s_cost in problem.getSuccessors(location):
				if s_location not in visited:
					frontier.push((s_location, path + [s_path], s_cost + cost))

	return []

def uniformCostSearch(problem):
	"""Search the node of least total cost first.
	You may need to pay close attention to util.py.
	Useful Reminder: Note that problem.getSuccessors(node) returns "step_cost".

	Key Point: If a node is already present in the queue with higher path cost
	(or higher priority), you'll update its cost (or priority)
	(Similar to pseudocode in figure 3.14 of your textbook.).
	Be careful, autograder cannot catch this bug.
	"""
	current_state = problem.getStartState()

	visited = set()
	frontier = util.PriorityQueue()

	frontier.push((current_state,[],0),0)

	while not frontier.isEmpty():
		location, path, cost = (frontier.pop())[0]

		if problem.isGoalState(location):
			return path

		if location not in visited:
			visited.add(location)

			for s_location, s_path, s_cost in problem.getSuccessors(location):
				if s_location not in visited:
					frontier.push((s_location, path + [s_path], s_cost),s_cost + cost)

	return []
	


def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0


def aStarSearch(problem, heuristic=nullHeuristic):
	"""
	Pay clos attention to util.py- specifically, args you pass to member functions.

	Key Point: If a node is already present in the queue with higher path cost
	(or higher priority), you'll update its cost (or priority)
	(Similar to pseudocode in figure 3.14 of your textbook.).
	Be careful, autograder cannot catch this bug.

	"""
	"Start of Your Code"
	current_state = problem.getStartState()

	visited = set()
	frontier = util.PriorityQueue()

	frontier.push((current_state,[],0),0)

	while not frontier.isEmpty():
		location, path, cost = (frontier.pop())[0]

		if problem.isGoalState(location):
			return path

		if location not in visited:
			visited.add(location)

			for s_location, s_path, s_cost in problem.getSuccessors(location):
				backwardCost = s_cost + cost
				forwardCost = heuristic(s_location, problem)
				total_cost = backwardCost + forwardCost
				frontier.push((s_location, path + [s_path], backwardCost), total_cost)
					

	return []
	"End of Your Code"



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
rdfs = RecursiveDepthFirstSearch
dls = depthLimitedSearch
rdls = RecursiveDepthLimitedSearch
astar = aStarSearch
ucs = uniformCostSearch
