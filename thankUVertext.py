import random
import sys
import math
import collections

class Node:
  def __init__(self, val):
    self.val = val
    self.visited = False
    self.neighbors = []

class DirectedGraph:
  def __init__(self):
    self.allNodes = []
    self.nodeMap = {}

  def getNode(self, val):
    return self.nodeMap.get(val)

  def addNode(self, nodeVal):
    node = Node(nodeVal)
    self.allNodes.append(node)
    self.nodeMap[nodeVal] = node
  
  def getAllNodes(self):
    return self.nodeMap

  def addDirectedEdge(self, first, second):
    if first == None or second == None:
      return
    else:
      try:
        if second not in self.allNodes[self.allNodes.index(first)].neighbors:
          self.allNodes[self.allNodes.index(first)].neighbors.append(second)
      except ValueError:
        return

  def removeDirectedEdge(self, first, second):
    if first == None or second == None:
      return

    idx2 = -1
    count = 0
    for node in self.allNodes[self.allNodes.index(first)].neighbors:
      if node.val == second.val:
        idx2 = count
        break
      count += 1
    if idx2 == -1:
      return
    try:
      self.allNodes[self.allNodes.index(first)].neighbors.pop(idx2)
    except ValueError:
      return

def createRandomDAGIter(n):
  g = DirectedGraph()
  lenRow = math.floor(math.sqrt(n))
  
  for i in range(n):
    g.addNode(i)

  for node in g.allNodes:
    if node.val + lenRow < n:
      #decide if we should add an edge between the node above
      if random.randint(0,2) < 2:
        g.addDirectedEdge(node, g.getNode(node.val+lenRow))
    if node.val % lenRow != lenRow - 1 and node.val + 1 < n:
      #decide if we should add an edge between the node to the right
      if random.randint(0,2) < 2:
        g.addDirectedEdge(node, g.getNode(node.val+1))

  return g

class TopSort:
  def Kahns(graph):
    inDegree = TopSort.initializeInDegreeMap(g)
    topSort = []
    queueInDegree0 = TopSort.addNodesWithoutDependenciesToQueue(inDegree, collections.deque())

    while len(queueInDegree0) > 0:
      currNode = queueInDegree0[0]
      topSort.append(currNode)
      
      for neighbor in g.nodeMap[currNode.val].neighbors:
        inDegree[neighbor] -= 1
        if inDegree[neighbor] == 0:
          queueInDegree0.append(neighbor)
      queueInDegree0.popleft()

    return topSort

  def initializeInDegreeMap(g):
    inDegree = {}
    for node in g.allNodes:
      inDegree[node] = 0

    for node in g.allNodes:
      for neighbor in node.neighbors:
        inDegree[neighbor] += 1
    
    return inDegree

  def addNodesWithoutDependenciesToQueue(inDegree, queue):
    for node in inDegree:
      if inDegree[node] == 0:
        queue.append(node)
        inDegree[node] = -1
    return queue

  def mDFS(graph):
    stack = []
    numNodes = len(g.allNodes)
    for vertex in g.allNodes:
      if vertex.visited != True:
        TopSort.mDFSHelper(vertex, stack)
    output = []
    while stack:
      output.append(stack.pop())
    
    return output
  
  def mDFSHelper(node, stack):
    node.visited = True
    for neighbor in node.neighbors:
      if neighbor.visited != True:
        TopSort.mDFSHelper(neighbor, stack) 
    stack.append(node)

g = DirectedGraph()
for i in range(16):
  g.addNode(i)

g.addDirectedEdge(g.getNode(0),g.getNode(4))
g.addDirectedEdge(g.getNode(0),g.getNode(1))
g.addDirectedEdge(g.getNode(1),g.getNode(2))
g.addDirectedEdge(g.getNode(2),g.getNode(6))
g.addDirectedEdge(g.getNode(3),g.getNode(7))
g.addDirectedEdge(g.getNode(0),g.getNode(4))
g.addDirectedEdge(g.getNode(4),g.getNode(5))
g.addDirectedEdge(g.getNode(4),g.getNode(8))
g.addDirectedEdge(g.getNode(5),g.getNode(6))
g.addDirectedEdge(g.getNode(5),g.getNode(9))
g.addDirectedEdge(g.getNode(6),g.getNode(10))
g.addDirectedEdge(g.getNode(7),g.getNode(11))
g.addDirectedEdge(g.getNode(8),g.getNode(12))
g.addDirectedEdge(g.getNode(9),g.getNode(13))
g.addDirectedEdge(g.getNode(11),g.getNode(15))
g.addDirectedEdge(g.getNode(13),g.getNode(14))

nodes = g.getAllNodes()
for val, node in nodes.items():
  print(val)
  for neighbor in node.neighbors:
    print("edge", neighbor.val)

out = TopSort.mDFS(g)
print("mDFS")
for node in out:
  print(node.val)

for node in g.getAllNodes().values():
  node.visited = False

out = TopSort.Kahns(g)
print("Kahns")
for node in out:
  print(node.val)
