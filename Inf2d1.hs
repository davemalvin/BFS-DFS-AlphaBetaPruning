-- Inf2d Assignment 1 2019-2020
-- Matriculation number: s1869672
-- {-# OPTIONS -Wall #-}


module Inf2d1 where

import Data.List (sortBy, elemIndices, elemIndex)
import ConnectFourWithTwist



{- NOTES:

-- DO NOT CHANGE THE NAMES OR TYPE DEFINITIONS OF THE FUNCTIONS!
You can write new auxillary functions, but don't change the names or type definitions
of the functions which you are asked to implement.

-- Comment your code.

-- You should submit this file when you have finished the assignment.

-- The deadline is the  10th March 2020 at 3pm.

-- See the assignment sheet and document files for more information on the predefined game functions.

-- See the README for description of a user interface to test your code.

-- See www.haskell.org for haskell revision.

-- Useful haskell topics, which you should revise:
-- Recursion
-- The Maybe monad
-- Higher-order functions
-- List processing functions: map, fold, filter, sortBy ...

-- See Russell and Norvig Chapters 3 for search algorithms,
-- and Chapter 5 for game search algorithms.

-}

-- Section 1: Uniform Search



-- The Node type defines the position of the agent on the graph.
-- The Branch type synonym defines the branch of search through the graph.
type Node = Int
type Branch = [Node]
type Graph= [Node]

numNodes::Int
numNodes = 4

-- FEEDBACK: There are a couple of cases your next can't handle
-- The next function should return all the possible continuations of input search branch through the graph.
-- Your function should return an empty list if the input search branch is empty.
-- This implementation of next function does not backtrace branches.
next::Branch -> Graph ->  [Branch]
-- EDGE CASE: If graph has no nodes in branch, return empty list.
next [] _ = []
-- EDGE CASE: If graph has no nodes, return empty list.
next _ [] = []
next branch g = [x : branch | x <- findEdge (editGraph g (head branch))]

-- FEEDBACK: Well done!
-- The checkArrival function should return true if the current location of the robot is the destination, and false otherwise.
checkArrival::Node -> Node -> Bool
checkArrival destination curNode
  | destination == curNode = True
  | otherwise = False

-- The explored function returns True if point is an element of exploredList.
explored::Node-> [Node] ->Bool
explored point exploredList = point `elem` exploredList

-- Section 3 Uniformed Search

-- FEEDBACK: Well done!
-- | Breadth-First Search
-- The breadthFirstSearch function should use the next function to expand a node,
-- and the checkArrival function to check whether a node is a destination position.
-- The function should search nodes using a breadth first search order.
breadthFirstSearch::Graph -> Node->(Branch ->Graph -> [Branch])->[Branch]->[Node]->Maybe Branch
-- If graph has no nodes, return Nothing.
breadthFirstSearch [] _ _ _ _ = Nothing
-- If search agenda is empty, return Nothing.
breadthFirstSearch g _ _ [] _ = Nothing
-- Since we are only considering DAG, a graph with 1 node will always return Nothing.
breadthFirstSearch [x] _ _ _ _ = Nothing
breadthFirstSearch g destination next branches exploredList
  -- If solution is found, return Just Branch.
  | checkArrival destination currNode = Just currBranch
  -- If current node of the search branch is not explored yet,
  -- then add it to exploredList and expand older nodes (by adding expanded current branch to the end of the search agenda).
  | not(explored currNode exploredList) = breadthFirstSearch g destination next (tail branches ++ next currBranch g) (currNode : exploredList)
  -- If current node has been explored, remove the current branch from search agenda.
  | otherwise = breadthFirstSearch g destination next (tail branches) exploredList
    where
      currNode = head (head branches)
      currBranch = head branches

-- FEEDBACK: There a few cases where your function does not find the solution even though it should.
-- | Depth-Limited Search
-- The depthLimitedSearch function is similiar to the depthFirstSearch function,
-- except its search is limited to a pre-determined depth, d, in the search tree.
depthLimitedSearch::Graph ->Node->(Branch ->Graph-> [Branch])->[Branch]-> Int->[Node]-> Maybe Branch
-- If graph has no nodes, return Nothing.
depthLimitedSearch [] _ _ _ _ _ = Nothing
-- If search agenda is empty, return Nothing.
depthLimitedSearch _ _ _ [] _ _ = Nothing
-- Since we are only considering DAG, a graph with 1 node will always return Nothing.
depthLimitedSearch [x] _ _ _ _ _ = Nothing
depthLimitedSearch g destination next branches  d exploredList
  -- If solution is found, return Just Branch.
  | checkArrival destination currNode = Just currBranch
  -- If current node of the search branch is not explored yet AND if current branch is less than or equal to limit d,
  -- then add it to exploredList and explore the child nodes of the current branch.
  | (not(explored currNode exploredList)) && (currBranchLength <= d) = depthLimitedSearch g destination next (next currBranch g ++ tail branches)  d (currNode : exploredList)
  -- Otherwise (including the case where current branch is greater than d), remove that branch from search agenda.
  | otherwise = depthLimitedSearch g destination next (tail branches)  d exploredList
    where
      currNode = head (head branches)
      currBranch = head branches
      currBranchLength = length currBranch


-- | Section 4: Informed search


-- | AStar Helper Functions

-- FEEDBACK: Well done!
-- | The cost function calculates the current cost of a trace. The cost for a single transition is given in the adjacency matrix.
-- The cost of a whole trace is the sum of all relevant transition costs.
cost :: Graph ->Branch  -> Int
cost gr [] = 0
cost gr [x] = 0
-- Use the auxiliary function editGraph to obtain the cost of a path from y to x, and recursively sum next path.
cost gr (x:y:ns) = (editGraph gr y)!!x + (cost gr (y:ns))

-- FEEDBACK: Well done!
-- | The getHr function reads the heuristic for a node from a given heuristic table.
-- The heuristic table gives the heuristic (in this case straight line distance) and has one entry per node. It is ordered by node (e.g. the heuristic for node 0 can be found at index 0 ..)
getHr:: [Int]->Node->Int
getHr hrTable node = hrTable!!node


-- FEEDBACK: Well done!
-- | A* Search
-- The aStarSearch function uses the checkArrival function to check whether a node is a destination position,
---- and a combination of the cost and heuristic functions to determine the order in which nodes are searched.
---- Nodes with a lower heuristic value should be searched before nodes with a higher heuristic value.

aStarSearch::Graph->Node->(Branch->Graph -> [Branch])->([Int]->Node->Int)->[Int]->(Graph->Branch->Int)->[Branch]-> [Node]-> Maybe Branch
aStarSearch [] _ _ _ _ _ _ _ = Nothing
aStarSearch _ _ _ _ _ _ [] _ = Nothing
aStarSearch g destination next getHr hrTable cost branches exploredList
  -- If solution is found, return Just Branch.
  | checkArrival destination currNode = Just currBranch
  | not(explored currNode exploredList) = aStarSearch g destination next getHr hrTable cost (next currBranch g ++ tail sortedBranches) (currNode : exploredList)
  | otherwise = aStarSearch g destination next getHr hrTable cost (tail sortedBranches) exploredList
    where
      sortedBranches = sortBranchesByCost g branches getHr hrTable cost
      currNode = head (head sortedBranches)
      currBranch = head sortedBranches

-- | Section 5: Games
-- See ConnectFourWithTwist.hs for more detail on  functions that might be helpful for your implementation.



-- | Section 5.1 Connect Four with a Twist


-- FEEDBACK: Well done!
-- The function determines the score of a terminal state, assigning it a value of +1, -1 or 0:
eval :: Game -> Int
eval game
  -- Checks if human player wins, then return 1, else if computer player wins, then return -1, otherwise game is a draw and 0 is returned.
  | terminal game && checkWin game humanPlayer = 1
  | terminal game && checkWin game compPlayer = -1
  | terminal game = 0

-- FEEDBACK: Your function does not always give the correct player.
-- | The alphabeta function should return the minimax value using alphabeta pruning.
-- The eval function should be used to get the value of a terminal state.
alphabeta:: Role -> Game -> Int
alphabeta  player game
  | player == maxPlayer = maxValue game (-2) (2)
  | player == minPlayer = minValue game (-2) (2)

maxValue :: Game -> Int -> Int -> Int
maxValue game alpha beta
  -- if TERMINAL-TEST(state) then return UTILITY(state)
  | terminal game = eval game
  -- v <- -infinity
  | otherwise = findMax (moves game 1) alpha beta (-2)

findMax :: [Game] -> Int -> Int -> Int -> Int
findMax [] alpha beta v = v
findMax (g:gs) alpha beta v
  -- if v' >= beta then return v'
  | v' >= beta = v'
  -- otherwise alpha <- MAX(alpha,v')
  | otherwise = findMax gs (max alpha v') beta v'
    where
      -- v' <- MAX(v, MIN-VALUE(RESULT(s,a),alpha,beta))
      v' = max v (minValue g alpha beta)


minValue :: Game -> Int -> Int -> Int
minValue game alpha beta
  -- if TERMINAL-TEST(state) then return UTILITY(state)
  | terminal game = eval game
  -- v <- +infinity
  | otherwise = findMin (moves game 0) alpha beta (2)

findMin :: [Game] -> Int -> Int -> Int -> Int
findMin [] alpha beta v = v
findMin (g:gs) alpha beta v
  -- if v' <= alpha then return v'
  | v' <= alpha = v'
  -- otherwise beta <- MIN(beta,v')
  | otherwise = findMin gs alpha (min beta v') v'
    where
      -- v' <- MIN(v, MAX-VALUE(RESULT(s,a),alpha,beta))
      v' = min v (maxValue g alpha beta)


-- | OPTIONAL!
-- You can try implementing this as a test for yourself or if you find alphabeta pruning too hard.
-- If you implement minimax instead of alphabeta, the maximum points you can get is 10% instead of 15%.
-- Note, we will only grade this function IF YOUR ALPHABETA FUNCTION IS EMPTY.
-- The minimax function should return the minimax value of the state (without alphabeta pruning).
-- The eval function should be used to get the value of a terminal state.
minimax:: Role -> Game -> Int
minimax player game=undefined

-- Auxiliary Functions
-- Include any auxiliary functions you need for your algorithms below.
-- For each function, state its purpose and comment adequately.
-- Functions which increase the complexity of the algorithm will not get additional scores

-- This will remove elements in g that are irrelevant in finding the possible continuations to branch
editGraph::Graph -> Int -> Graph
editGraph g idx = take numNodes (drop (numNodes*idx) g)

-- This will return the possible continuations to branch as a list of nodes.
-- It works hand-in-hand with the 'editGraph' function, where we check if each elem in the 'edited' graph is not equal to 0, indicating an edge to that node.
findEdge::Graph -> [Node]
findEdge g = [x | x <- [0..numNodes-1], g!!x /= 0]

-- Compares the heuristc value of two branches as tuples.
compareCost :: (Branch, Int) -> (Branch, Int) -> Ordering
compareCost (_,cost1) (_,cost2) = compare cost1 cost2

-- HOW sortBy WORKS:
-- sortBy (\(a,_) (b,_) -> compare a b) [(2, "world"), (4, "!"), (1, "Hello")]
-- [(1,"Hello"),(2,"world"),(4,"!")]

-- Sorts the search agenda based on the total cost (heuristic value + path cost).
sortBranchesByCost::Graph->[Branch]->([Int]->Node->Int)->[Int]->(Graph->Branch->Int)->[Branch]
sortBranchesByCost g branches getHr hrTable cost = [ branch | (branch,tCost) <- sortBy compareCost (zip branches totalCost) ]
  where
    currNode = head (head branches)
    currBranch = head branches
    -- This is a list of the total costs of each **corresponding** branch in branches.
    totalCost = [ (getHr hrTable (head currBranch)) + (cost g currBranch) | currBranch <- branches ]
