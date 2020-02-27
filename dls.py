from path_state import make_child_node, h_function
from collections import deque
from structures import Result
import time
import heapq

def dls(problem, board, limit, verbose=False):
    #print("\n---------------------------------------\n")
    #print(f"Running DFS with\nState: {problem.state}\nGoal State: {problem.goal}\nLimit: {limit}...")
    
    start = time.perf_counter()
    
    frontier = [problem]
    frontier_size = len(frontier)
    gen_nodes = 0
    explored = set()
    mapp = {}
     
    if problem.str_state == problem.goal_str:
        end_prem = time.perf_counter()
        
        return Result(problem, "success", frontier_size, problem.depth, gen_nodes, end_prem - start)
    
    while frontier:
        node = frontier.pop()
        #print(len(frontier))
        #print(f"{node.state}")
        if node.depth <= limit:
            
            if node.str_state not in explored:
                explored.add(node.str_state)
                mapp[node.str_state] = node
                
                if node.str_state == node.goal_str:
                    end_final = time.perf_counter()
                    return Result(node, "success", frontier_size, node.depth, gen_nodes, end_final - start)
                
                children = reversed(make_child_node(node, node.goal, board))
                for child in children:
                    gen_nodes = gen_nodes + 1
                    frontier.append(child)
                if len(frontier) > frontier_size:
                    frontier_size = len(frontier)
            
            elif node.str_state in mapp and node.path_cost < mapp[node.str_state].path_cost:
                explored.remove(node.str_state)
                frontier.append(node)
    
    end_failed = time.perf_counter()    
    return Result(None, "failed", frontier_size, limit, gen_nodes, end_failed - start)

