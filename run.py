from structures import Node, NodeFCost, NodeGCost, Board
from path_state import path, path_to_goal
from generator import all_in_one
from bfs import bfs
from ucs import ucs
from dls import dls
from ids import ids
from gbfs import gbfs
from a_star import a_star

goal_state = []
start_state = []

def set_goal(size):
    print("Goal State: ")
    for x in range(0, size*size):
        x = int(input())
        goal_state.append(x)

def get_start_node(size):        
    print("Start State: ")
    for x in range(0, size*size):
        x = int(input())
        start_state.append(x)
    start_node = Node(start_state, goal_state, None, None, 0, 0)
    return start_node

def get_board_size():
    board_size = int(input("Board size: "))
    return board_size

def show_statistics(node):
    print(f"Nodes Generated: {node.gen_nodes}")
    print(f"Max Frontier Size: {node.frontier_size}")
    print(f"Max Search Depth: {node.max_depth}")
    print(f"Solution found at depth: {node.node.depth}")
    print(f"Path Cost: {node.node.path_cost}")
    print(f"Elapsed Time: {node.elapsed_time} seconds\n")
    
def main():
    
    #board_size = int(input("Board size: "))
    bsize = int(get_board_size())
    board = Board(bsize)
    
    set_goal(bsize)
    start_node = get_start_node(bsize)
    
    running = True
    choice = 0
    dls_limit = None
    while running:
        print("\n---------------------------------------------------")
        print("Choose and algorithm (1-6) or press 0 to quit:")
        print("1. Breadth-First Search (BFS)")
        print("2. Uninformed Cost Search (UCS)")
        print("3. Depth Limited Search (DLS)")
        print("4. Iterativee Deepening Depth-First Search (IDS)")
        print("5. Greedy Best-First Search (GBFS)")
        print("6. A* Search")
        print("7. Generate Graphs on 8-Puzzle (time, nodes generated, cost)")
        try:
            choice = int(input("Choice: "))
        except:
            print("Only interger input allowed")
            continue
        
        if choice == 0:
            break
        
        elif choice == 1:
            print(f"Running BFS on: {start_state}")
            print(f"Goal: {goal_state}\n...")
            result = bfs(start_node, board)
            dls_limit = result.max_depth
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 2:
            print(f"Running UCS on: {start_state}")
            print(f"Goal: {goal_state}\n...")
            start_node_ucs = NodeGCost(start_state, goal_state, None, None, 0, 0)
            result = ucs(start_node_ucs, board, verbose=False)
            dls_limit = result.max_depth
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 3:
            if dls_limit is None:
                print("DLS limit not set by BFS yet")
                dls_limit = int(input("Limit: "))
            else:
                print(f"BFS set limit to {dls_limit}")
            print(f"Running DLS on: {start_state}")
            print(f"Goal: {goal_state}")
            print(f"limit: {dls_limit}\n...")
            
            result = dls(start_node, board, dls_limit)
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 4:
            print(f"Running IDS on: {start_state}")
            print(f"Goal: {goal_state}")
            result = ids(start_node, board, 0)
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 5:
            print(f"Running GBFS on: {start_state}")
            print(f"Goal: {goal_state}\n...")
            result = gbfs(start_node, board)
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 6:
            print(f"Running A* on: {start_state}")
            print(f"Goal: {goal_state}\n...")
            start_node_a = NodeFCost(start_state, goal_state, None, None, 0, 0) 
            result = a_star(start_node_a, board)
            if result.verdict == 'success':
                print("Goal State Found!")
                show_statistics(result)
                path(result.node)
                path_to_goal()
            elif result.verdict == 'failed':
                print("Goal Not Found")
        
        elif choice == 7:
            all_in_one(goal_state,board)

if __name__ == '__main__':
    main()
            