from filecmp import cmp
from math import sqrt
from collections import deque
from collections.abc import Iterator
from functools import cmp_to_key
import time
from  collections import deque  


actions = {
  1: (1, 0, 0),
 2: (-1, 0, 0),
 3: (0, 1, 0),
 4: (0, -1, 0),
 5: (0, 0, 1),
 6: (0, 0, -1),
 7: (1, 1, 0),
 8: (1, -1, 0),
 9: (-1, 1, 0),
 10: (-1, -1, 0),
 11: (1, 0, 1),
 12: (1, 0, -1),
 13: (-1, 0, 1),
 14: (-1, 0, -1),
 15: (0, 1, 1),
 16: (0, 1, -1),
 17: (0, -1, 1),
 18: (0, -1, -1),
}

# class Node:
#   def __init__(self, loc, cost):
#     self.loc = loc
#     self.cost = cost
#     self.next = None


# class SinglyLinkedList:
#   def __init__(self, head=None):
#     if head is not None:
#       self.head = head
#       self.tail = head
#     else:
#       self.head = None
#       self.tail = None
  
#   def add(self, loc, cost):
#     if self.head is None:
#       self.head = Node(loc, cost)
#       self.tail = self.head
#     else:
#       self.tail.next = Node(loc, cost)
#       self.tail = self.tail.next


def is_valid_dimension(my_location, grid_dimension):
  for i in range(3):
    if my_location[i] < 0 or my_location[i] > grid_dimension[i]:
      return False
  return True


def bfs(start, end, locations, dimensions):
  queue = deque()
  visited =  {}
  open = set()
  cost = 0
  result = []
  queue.append((start, None, 0))
  while queue:
    node, parent, cost = queue.popleft()
    open.add(node)
    if node == end:
      grand_parent, grand_cost, grand_movement_cost = visited[parent]
      visited[node] = (parent, cost, cost - grand_cost)
      break

    movements = locations.get(node)

    for action in movements:
      new_loc = move(node, actions.get(action))

      if not is_valid_dimension(new_loc, dimensions):
        continue

      movement_cost = cost + 1
      if new_loc in open or new_loc in visited:
        continue
      else:
        open.add(new_loc)
        queue.append((new_loc, node, movement_cost))
    if not visited:
      visited[node] = (parent, cost, 0)
     
    else:
      visited[node] = (parent, cost, movement_cost - cost)
   
  

  if not queue:
    return "FAIL"
  
  cost = visited[end][1]
  cost_list = []
  while end is not None:
    result.insert(0, end)
    parent_state = visited[end]
    end = parent_state[0]
    cost_list.insert(0, parent_state[2])

  return {"total_cost": cost, "node_list": result, "cost_list": cost_list}


def queue_function(node1, node2):
  return node2[2] - node1[2]


def ucs(start, end, locations, dimensions):
  queue = deque()
  visited =  {}
  cost = 0
  result = []
  open = set()

  queue.append((start, None, 0))
  while queue:
    node, parent, cost = queue.pop()
 
    if node == end:
      grand_parent, grand_cost, grand_movement_cost  = visited[parent]
      visited[node] = (parent, cost, cost - grand_cost)
      break

    movements = locations[node]
    for action in movements:
      if action <= 6:
        movement_cost = 10
      else: 
        movement_cost = 14

      new_loc = move(node, actions.get(action))

      if not is_valid_dimension(new_loc, dimensions):
        continue
      movement_cost += cost
      if new_loc == parent:
        continue
      for item in queue:
        if item[0] == new_loc:
          if item[2] > (cost + movement_cost):
            queue.remove(item)
            queue.append((new_loc, node, movement_cost))
            queue = sorted(queue, key=cmp_to_key(queue_function))
      if new_loc in visited:
        for item, values in list(visited.items()):
          if item == new_loc:
            if values[1] > (cost + movement_cost):
              visited.pop(item)
              queue.append((new_loc, node, movement_cost))
              queue = sorted(queue, key=cmp_to_key(queue_function))
      else:
        queue.append((new_loc, node, movement_cost))
        queue = sorted(queue, key=cmp_to_key(queue_function))
    if not visited:
      visited[node] = (parent, cost, 0)
    else:
      grand_parent, grand_cost, grand_movement_cost  = visited[parent]
      visited[node] = (parent, cost, cost - grand_cost)
  
  if not queue:
    return "FAIL"
 
  cost = visited[end][1]
  cost_list = []
  while end is not None:
    result.insert(0, end)
    parent_state = visited[end]
    end = parent_state[0]
    cost_list.insert(0, parent_state[2])

  return {"total_cost": cost, "node_list": result, "cost_list": cost_list}

def heuristics(start, end):
  res = 0
  for i in range(0, 3):
    res += (abs((start[i] - end[i]) * 10) ** 2)
  return sqrt(res) 


def a_star(start, end, locations, dimensions):
  queue = deque()
  visited = {}
  cost = 0
  result = []
  val = heuristics(start, end)
  queue.append((start, None, val, 0))
  while queue:
    node, parent, prediction, cost = queue.pop()
 
    if node == end:
      grand_parent, grand_cost, grand_prediction, grand_movement_cost = visited[parent]
      visited[node] = (parent, cost, 0, cost - grand_cost)
      break
    movements = locations[node]
    for action in movements:
      if action <= 6:
        movement_cost = 10
      else: 
        movement_cost = 14
      new_loc = move(node, actions.get(action))
      if not is_valid_dimension(new_loc, dimensions):
        continue

      movement_cost += cost
      if new_loc == parent:
        continue

      if new_loc in visited:
        continue
      else:
        a_val = heuristics(new_loc, end)
        queue.append((new_loc, node, a_val + movement_cost, movement_cost))
        queue = sorted(queue, key=cmp_to_key(queue_function))
    if not visited:
      visited[node] = (parent, cost, prediction, 0)
    else:
      grand_parent, grand_cost, grand_prediction, grand_movement_cost = visited[parent]
      visited[node] = (parent, cost, prediction, cost - grand_cost)
  if not queue:
    return "FAIL"
  costs_list = []
  cost = visited[end][1]
  while end is not None:
    result.insert(0, end)
    parent_state = visited[end]
    end = parent_state[0]
    costs_list.insert(0, parent_state[3])

  return {"total_cost": cost, "node_list": result, "cost_list": costs_list}



def initialize(file_input):
  if(file_input["instruction"] == "BFS\n"):
    path = bfs(file_input["start"], file_input["end"], file_input["locations"], file_input['dimensions'])
  elif(file_input["instruction"] == "UCS\n"):
    path = ucs(file_input["start"], file_input["end"], file_input["locations"], file_input["dimensions"])
  elif(file_input["instruction"] == "A*\n"):
    path = a_star(file_input["start"], file_input["end"], file_input["locations"], file_input["dimensions"])
  file = open("output.txt", "w")
  if path == "FAIL":
    file.write("FAIL")
  else:
    write_file(path, file)
    

def write_file(path, file):
  file.write(str(path["total_cost"]) + "\n")
  file.write(str(len(path["node_list"])) + "\n")
  for item, cost in zip(path["node_list"], path["cost_list"]):
    for i in item:
      file.write(str(i) + " ")
    file.write(str(cost))
    file.write("\n")
  file.close()
    

 
def main():
  start = time.time()
  file_input = readFile()
  initialize(file_input)
  end = time.time()
  spent = round(end - start, 4)
  print("It took: ", spent, "secs")




def convert_to_locations(string):
  container = []
  for point in string.split():
    container.append(int(point))
  return tuple(container)

def move(start_from, move_var):
  new_loc = []
  for i in range(0, 3):
    new_loc.append(start_from[i] + move_var[i])
  return tuple(new_loc)

def convert_to_grid_points(file, n):
  container = []
  location_map = {}
  for x in range(0, n):
    line = file.readline()
    for token in line.split():
      container.append(int(token))
    location = tuple(container[0:3])
    location_map[location] = []
    action_list = container[3:]
    for action in action_list:
      location_map[location].append(action)
    container.clear()
  return location_map



def readFile():
  with open("sample/input6.txt") as file:
    container = []
    grid_point_actions = []
    instruction = file.readline()
    dimensions = convert_to_locations(file.readline())
    starting_point = file.readline()
    ending_point = file.readline()
    number_grids = int(file.readline())
    start = convert_to_locations(starting_point)
    end = convert_to_locations(ending_point)
    locations = convert_to_grid_points(file, number_grids)

    return {"locations": locations, "start": start, "end": end, "instruction": instruction, "dimensions": dimensions}
 


if __name__ == "__main__":
  main()