from queue import PriorityQueue
from time import time

#Prints the board state in a nice 8 puzzle format
def printPuzzel(board):
    for i in board:
        for cell in i:
            print(cell, end=" ")
        print()

# Locates and returns a pair of (row, col) indices where the blank space is found
def findEmptyIndices(board):
    for i in range(len(board)):
        for j in range(len(board[i])):
            if board[i][j] == '0':
                return [i, j]

# Counts and returns the number of misplaced tiles using the current state compared to the goal state
def computeMisplacedTiles(cur_state, goal_state):
    total_tiles = 0
    for i in range(len(cur_state)):
        for j in range(len(cur_state[i])):
            current_state_tile = cur_state[i][j]
            goal_state_tile = goal_state[i][j]
            if current_state_tile != goal_state_tile:
                total_tiles += 1
    # Subtract 1 to ignore the blank space being counted
    return total_tiles - 1

# Calculates and returns the sum of Manhattan distances between each current state tile and goal state tile
def computeManhattan(cur_state, goal_state):
    total_distance = 0
    # Create a mapping between tile number and indices n -> (x, y)
    cur_mappings = {}
    goal_state_mappings = {}
    # Iterate through entire board and store a map between the tile number and indices
    for i in range(len(cur_state)):
        for j in range(len(cur_state[i])):
            cur_tile = cur_state[i][j]
            goal_tile = goal_state[i][j]
            cur_mappings[cur_tile] = (i, j)
            goal_state_mappings[goal_tile] = (i, j)
    # Calculate Manhatten distances for non-empty spaces
    del cur_mappings['0']
    # Iterate through mappings and calculate the Manhatten distance between the current state index pair and goal state index pair
    for tile, indices in cur_mappings.items():
        x1, y1 = indices
        x2, y2 = goal_state_mappings[tile]
        distance = abs(x1 - x2) + abs(y1 - y2)
        total_distance += distance
    return total_distance

class Node:
    def __init__(self, board, g_n=0):
        self.state = board
        self.g_n = g_n
        self.parent = None

    # Returns a list of next possible Nodes depending on the current state
    def getPossibleStates(self):
        possible_states = []
        next_g_n = self.g_n + 1

        # Find out where the indices for the blank space is located in the board
        blank_index_row, blank_index_col = findEmptyIndices(self.state)

    # Check if we can move to the up
        if blank_index_row > 0:
            next_state = [row.copy() for row in self.state]
            up = blank_index_row - 1
            next_state[blank_index_row][blank_index_col], next_state[up][blank_index_col] = \
            next_state[up][blank_index_col], next_state[blank_index_row][blank_index_col]
            node = Node(next_state, next_g_n)
            possible_states.append(node)

        # Check if we can move down
        if blank_index_row < 2:
            next_state = [row.copy() for row in self.state]
            down = blank_index_row + 1
            next_state[blank_index_row][blank_index_col], next_state[down][blank_index_col] = \
            next_state[down][blank_index_col], next_state[blank_index_row][blank_index_col]

            node = Node(next_state, next_g_n)
            possible_states.append(node)

        # Check if we can move to the left
        if blank_index_col > 0:
            next_state = [row.copy() for row in self.state]
            left = blank_index_col - 1
            next_state[blank_index_row][blank_index_col], next_state[blank_index_row][left] = \
            next_state[blank_index_row][left], next_state[blank_index_row][blank_index_col]

            node = Node(next_state, next_g_n)
            possible_states.append(node)

        # Check if we can move to the right
        if blank_index_col < 2:
            next_state = [row.copy() for row in self.state]
            right = blank_index_col + 1
            next_state[blank_index_row][blank_index_col], next_state[blank_index_row][right] = \
            next_state[blank_index_row][right], next_state[blank_index_row][blank_index_col]
            node = Node(next_state, next_g_n)
            possible_states.append(node)
        return possible_states

# Trace back and store solution using the parent nodes
def getFinalPath(node):
    states = []
    max_depth = 0
    while node.parent != None:
        states.append(node.state)
        node = node.parent
        max_depth += 1
    # Reverse list to print in order
    return states[::-1], max_depth

def aStar(heuristic):
    current_node = Node(initial_state)
    nodes_expanded = 0
    count = 1
    state = tuple(map(tuple, initial_state))
    explored_hash = {state}
    frontier = PriorityQueue()
    max_num_frontier_nodes = 0
    goal_state = [
        ['1', '2', '3'],
        ['4', '5', '6'],
        ['7', '8', '0']
    ]
    print("Expanding state")
    printPuzzel(current_node.state)
    print()
    # If our initial state is actually our goal state, we're done
    if current_node.state == goal_state:
        print("Goal Reached!\n")
        print(f"Total number of nodes expanded: {nodes_expanded}")
        print(f"The maximum number of nodes in the queue: {max_num_frontier_nodes}.\n")
        return [current_node.state], 0

    # Calculate our f(n) = g(n) + h(n)
    # If it is uniformcost Search set h(n) to zero else compute h(n) depending on the heuristics
    if heuristic == 0:
        h_n = 0
    else:
        h_n = heuristic(current_node.state, goal_state)
    f_n = current_node.g_n + h_n

    # Use f_n as the primary comparator for the frontier PriorityQueue
    element = (f_n, count, current_node)
    frontier.put(element)

    # Continue looping until the frontier is not empty
    while not frontier.empty():
        # Update the largest number of nodes in the frontier
        max_num_frontier_nodes = max(max_num_frontier_nodes, frontier.qsize())

        # Dequeue and extract the h(n) value and the Node
        top = frontier.get()
        current_f_n = top[0]
        current_node = top[2]
        if heuristic == 0:
            current_h_n = 0
        else:
            current_h_n = current_f_n - current_node.g_n

        explored_hash.add(tuple(map(tuple, current_node.state)))
        # If goal state is reached, return the solution path
        if current_node.state == goal_state:
            print("Goal Reached!\n")
            print(f"Total number of nodes expanded: {nodes_expanded}")
            print(f"The maximum number of nodes in the queue: {max_num_frontier_nodes}.\n")
            return getFinalPath(current_node)

        # Else continue expanding the frontier nodes by checking the possible states.
        print(f"The best state to expand with g(n) = {current_node.g_n} and h(n) = {current_h_n} is...")
        printPuzzel(current_node.state)
        print("Expanding node...\n")

        # Find all the possible states to move to with our current state
        possible_states = current_node.getPossibleStates()
        for node in possible_states:
            count += 1
            state = tuple(map(tuple, node.state))
            # Add to the frontier if we haven't seen this state yet
            if state not in explored_hash:
                if heuristic == 0:
                    h_n = 0
                else:
                    h_n = heuristic(node.state, goal_state)
                f_n = node.g_n + h_n

                # Assigning node to parent and then push into the frontier
                node.parent = current_node
                element = (f_n, count, node)
                frontier.put(element)

        # Increment the number of nodes expanded
        nodes_expanded += 1

if __name__ == '__main__':

    print("Enter the Initial state for 8 puzzle game, use zero to represent the blank space.")

    first_row = input("Enter first three space separated numbers: ")
    second_row = input("Enter second three space separated numbers: ")
    third_row = input("Enter third three space separated numbers: ")

    first_row_tiles = first_row.split()
    second_row_tiles = second_row.split()
    third_row_tiles = third_row.split()
    initial_state = [first_row_tiles, second_row_tiles, third_row_tiles]
    # Prompt which search algorithm to perfom
    select = input(
        "Enter your choice of algorithm.\n"
        "1 - Uniform Cost Search\n"
        "2 - A* with the Misplaced Tile heuristic\n"
        "3 - A* with the Manhattan distance heuristic\n")
    # start timer and run the selected algorithm
    start = time()
    if select == "1":
        print("Search using uniform cost...\n")
        solution_path, solution_depth = aStar(0)
    elif select == "2":
        print("Search using A* misplaced tile...\n")
        solution_path, solution_depth = aStar(computeMisplacedTiles)
    elif select == "3":
        print("Search using A* Manhattan distance...\n")
        solution_path, solution_depth = aStar(computeManhattan)
    # End timer
    end = time()

    # Print solution
    print("Solution path is...\n")
    for state in solution_path:
        printPuzzel(state)
        print()
    print(f'Solution depth: {solution_depth}')

    # Print how long it took to solve
    time_taken = end - start
    print(f"Search completed in {time_taken} seconds.")
