import random
import sys
import math
import heapq

class GridNode:
  def __init__(self, x, y, nodeVal):
    self.val = nodeVal
    self.coordinates = (x,y)
    self.visited = False
    self.neighbors = []

  def __lt__(self, other):
    in1=self.val
    in2=other.val
    if in1 < in2:
      return in1
    return in2

class GridGraph:
  def __init__(self):
    self.allNodes = []
    self.nodeMap = {}

  def getNode(self, val):
    return self.nodeMap.get(val)

  def getAllNodes(self):
    return self.nodeMap

  def addGridNode(self, x, y, nodeVal):
    if nodeVal in self.nodeMap:
      print("duplicate node value")
      return
    node = GridNode(x, y, nodeVal)
    self.allNodes.append(node)
    self.nodeMap[nodeVal] = node
  
  def isNeighbor(first, second):
    firstX = first.coordinates[0]
    firstY = first.coordinates[1]
    secondX = second.coordinates[0]
    secondY = second.coordinates[1]

    if (firstX + 1 == secondX and firstY == secondY) or (firstX == secondX and firstY + 1 == secondY) or (firstX-1 == secondX and firstY == secondY) or (firstX == secondX and firstY - 1 == secondY):
        return True
    return False
  
  def addUndirectedEdge(self, first, second):
    if first == None or second == None:
      print("first and/or second node is None")
      return 
    
    if first == second:
      print("first and second are equal, no edges to itself")
      return

    if not GridGraph.isNeighbor(first, second):
      print("node is not a neighbor")
      return

    self.nodeMap[first.val].neighbors.append(second)
    self.nodeMap[second.val].neighbors.append(first)
    
  def removeUndirectedEdge(self, first, second):
    if first == None or second == None:
      print("first and/or second node is None")
      return 
    
    if first == second:
      print("first and second are equal, no edges to itself")
      return

    if not GridGraph.isNeighbor(first, second):
      print("node is not a neighbor")
      return

    if first in second.neighbors and second in first.neighbors:
      self.nodeMap[first.val].neighbors.remove(second)
      self.nodeMap[second.val].neighbors.remove(first)
    else:
      print("no edge exists between first and second")
    
def createRandomGridGraph(n):
  g = GridGraph()
  numNodes = 0

  for x in range(n):
    for y in range(n):
      g.addGridNode(x, y, numNodes)
      numNodes += 1

  for node in g.allNodes:
    if node.val + n < n*n:
      #decide if we should add an edge between the node above
      #making it an 90% chance because we want to make sure there's a connection to the end
      if random.randint(0,9) < 9:
        g.addUndirectedEdge(node, g.getNode(node.val+n))
    if node.val % n != n - 1 and node.val + 1 < n*n:
      #decide if we should add an edge between the node to the right
      if random.randint(0,9) < 9:
        g.addUndirectedEdge(node, g.getNode(node.val+1))
  
  return g

def minDist(distances, visited):
  ans = None
  m = math.inf
  for curr in distances.keys():
    if visited[curr] != True and distances[curr] <= m:
      m = distances[curr]
      ans = curr
  return ans

def heuristic(start, end):
  #manhattan distance
  xdiff = end.coordinates[0] - start.coordinates[0]
  ydiff = end.coordinates[1] - start.coordinates[1]
  return abs(xdiff) + abs(ydiff) 

def astar(sourceNode, destNode):
  if not sourceNode or not destNode:
    return []

  mapDistances = {}
  mapVisited = {}
  mapParents = {}
  mapHeuristicDistances = {}
  min_dist = []
  
  mapDistances[sourceNode] = 0
  mapParents[sourceNode] = None
  mapHeuristicDistances[sourceNode] = heuristic(sourceNode, destNode)

  mapVisited[destNode] = False
  mapVisited[sourceNode] = False
  heapq.heappush(min_dist, (mapHeuristicDistances[sourceNode], sourceNode))
  while min_dist:
    # “Finalize” curr.
    currDistance, curr = heapq.heappop(min_dist)
    if mapVisited[curr] == True:
      continue
    mapVisited[curr] = True
    if mapVisited[destNode] == True:
      break
    # Iterate over its neighbors, “relax” each neighbor:
    for neighbor in curr.neighbors:
      if neighbor not in mapDistances:
        mapDistances[neighbor] = math.inf
        mapVisited[neighbor] = False
        mapParents[neighbor] = None

      if mapVisited[neighbor] != True:
        if mapDistances[curr] + 1 < mapDistances[neighbor]:
          mapParents[neighbor] = curr
          mapDistances[neighbor] = mapDistances[curr] + 1
          mapHeuristicDistances[neighbor] = mapDistances[neighbor] + heuristic(neighbor, destNode)
          heapq.heappush(min_dist, (mapHeuristicDistances[neighbor], neighbor))
  
  path = []
  if curr != destNode:
    return path
  path.insert(0, curr.val)
  while mapParents[curr] != None:
    path.insert(0, mapParents[curr].val)
    curr = mapParents[curr]
  return path

if __name__ == '__main__':
  g = createRandomGridGraph(10)

  nodes = g.getAllNodes()
  for node in nodes.values():
    print(node.val)
    for edge in node.neighbors:
      print("edge", edge.val)

  nodes = g.getAllNodes()
  for node in nodes.values():
    print(node.val)
    for edge in node.neighbors:
      print("edge", edge.val)

  path = astar(g.getNode(0),g.getNode(99))
  print(path)

  print()
  #edgextra credit !!!!!!
  print("Number of Nodes finalized: ", len(path))
