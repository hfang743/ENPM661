import numpy as np
import os

class Node:         # Define variables
    def __init__(self, node_index, data, parent, act):
        self.data = data
        self.parent = parent
        self.act = act
        self.node_index = node_index
#       self.p = p



def Node_Initial():       # Get initial 3x3 matrix input in column-wise order
    print("Enter elements from 0-8 in column-wise order:")
    initial_state = np.ones(9)
    for i in range(9):
        state = int(input('Enter the ' + str(i + 1) + " number: "))
        initial_state[i] = np.array(state)
    return np.reshape(initial_state, (3, 3), order='F')



def BlankTileLocation(CurrentNode):  # Use two 'For loops' to locate '0' in 3X3 matrix
    for i in range(0, 3):
        for j in range(0, 3):
            i, j = np.where(CurrentNode == 0)
            i = int(i)
            j = int(j)
            return i, j



def ActionMoveLeft(CurrentNode):  # Function to move the blank tile left
    i, j = BlankTileLocation(CurrentNode)
    if j == 0:
        return None
    else:
        temp_arr = np.copy(CurrentNode)
        temp = temp_arr[i, j - 1]
        temp_arr[i, j] = temp
        temp_arr[i, j - 1] = 0
        return temp_arr

def ActionMoveRight(CurrentNode):   # Function to move the blank tile right
    i, j = BlankTileLocation(CurrentNode)
    if j == 2:
        return None
    else:
        temp_arr = np.copy(CurrentNode)
        temp = temp_arr[i, j + 1]
        temp_arr[i, j] = temp
        temp_arr[i, j + 1] = 0
        return temp_arr

def ActionMoveUp(CurrentNode):     # Function to move the blank tile up
    i, j = BlankTileLocation(CurrentNode)
    if i == 0:
        return None
    else:
        temp_arr = np.copy(CurrentNode)
        temp = temp_arr[i - 1, j]
        temp_arr[i, j] = temp
        temp_arr[i - 1, j] = 0
        return temp_arr

def ActionMoveDown(CurrentNode):      # Function to move the blank tile down
    i, j = BlankTileLocation(CurrentNode)
    if i == 2:
        return None
    else:
        temp_arr = np.copy(CurrentNode)
        temp = temp_arr[i + 1, j]
        temp_arr[i, j] = temp
        temp_arr[i + 1, j] = 0
        return temp_arr

def move_tile(Action, CurrentNode):  #Define actions showing for blank tile moving 
    if Action == 'left':
        return ActionMoveLeft(CurrentNode)
    if Action == 'right':
        return ActionMoveRight(CurrentNode)
    if Action == 'up':
        return ActionMoveUp(CurrentNode)
    if Action == 'down':
        return ActionMoveDown(CurrentNode)
    else:
        return None

def path(node):  #Create a path list and add parent nodes explored 
    p = []
    p.append(node)
    parent_node = node.parent
    while parent_node is not None:
        p.append(parent_node)
        parent_node = parent_node.parent
    return list(reversed(p))

def exploring_nodes(node):   #Function to define the solution state
    print("Exploring...")
    actions = ["down", "up", "left", "right"]
    goal_node = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])
    node_q = [node]
    final_nodes = []
    visited = []
    final_nodes.append(node_q[0].data.tolist())
    node_counter = 0

    while node_q:
        current_root = node_q.pop(0)
        if current_root.data.tolist() == goal_node.tolist():
            print("Solution found")
            return current_root, final_nodes, visited

        for move in actions:
            temp_data = move_tile(move, current_root.data)
            if temp_data is not None:
                node_counter += 1
                child_node = Node(node_counter, np.array(temp_data), current_root, move)  # Create a child node

                if child_node.data.tolist() not in final_nodes:  # Add the child node data in node list
                    node_q.append(child_node)
                    final_nodes.append(child_node.data.tolist())
                    visited.append(child_node)
                    if child_node.data.tolist() == goal_node.tolist():
                        print("Solution found")
                        return child_node, final_nodes, visited
    return None, None, None

def write_nodes(explored):   # Write all the nodes explored by BFS
    if os.path.exists("Nodes.txt"):
        os.remove("Nodes.txt")
    f = open("Nodes.txt", "w")
    for nodes in explored:
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                f.write(str(nodes[j][i]) + " ")
        f.write("\n")
    f.close()

# def write_path(paths):  # Write the path in the text file
#     if os.path.exists("nodePath.txt"):
#         os.remove("nodePath.txt")
#     f = open("nodePath.txt", "w")
#     for path in paths:
#         for i in range(len(path)):
#             f.write(str(path[i]) + " ")
#         f.write("\n")
#     f.close()

def write_node_info(info):  # Write info about the parent nodes from the final path
    if os.path.exists("Node_info.txt"):
        os.remove("Node_info.txt")
    f = open("Node_info.txt", "w")
    for n in info:
        if n.parent is not None:
            f.write(str(n.node_index) + " " + str(n.parent.node_index) + "\n")
    f.close()


def check_solvability(c): # function to check solvability
    arr = np.reshape(c, 9)
    counter_states = 0
    for i in range(9):
        if not arr[i] == 0:
            check_elem = arr[i]
            for x in range(i + 1, 9):
                if check_elem < arr[x] or arr[x] == 0:
                    continue
                else:
                    counter_states += 1
    if counter_states % 2 == 0:
        print("The puzzle is solvable, generating path")
    else:
        print("The puzzle is insolvable, please enter a new set of nodes")

def print_states(list_final):  # To print the final states on the console
    print("printing out final solution")
    for l in list_final:
        print("Move : " + str(l.act) + "\n" + "Result : " + "\n" + str(l.data) + "\t" + "node number:" + str(l.node_index))

g = Node_Initial()
check_solvability(g)
root = Node(0, g, None, None)

goal, e, v = exploring_nodes(root)

# Print and write the final output
if goal is None and e is None and v is None:
    print("Cannot find the solution")
else:
    print_states(path(goal))
    # write_path(p)
    write_nodes(e)
    write_node_info(v)

