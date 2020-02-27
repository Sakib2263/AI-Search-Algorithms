from path_state import make_child_node, h_function
from collections import deque
from structures import Result
import time
import heapq

def bfs(problem, board, verbose=False):
    start = time.perf_counter()
    
    with open("log/bfs.txt", "w") as file_obj:
        
        max_depth = 0
        gen_nodes = 0
        
        frontier = deque([problem])
        frontier_size = len(frontier)
        explored = set()
        
        if problem.str_state == problem.goal_str:
            end_prem = time.perf_counter()
            return Result(problem, "success", frontier_size, problem.depth, gen_nodes, end_prem - start)
        
        while frontier:
            if verbose:
                print(f"Frontier before extraction: {len(frontier)}")
                file_obj.write(f"Frontier before extraction: {len(frontier)}\n")
            node = frontier.popleft()
            if verbose:
                print(f"Frontier after extraction: {len(frontier)}")
                file_obj.write(f"Frontier after extraction: {len(frontier)}\n")
            explored.add(node.str_state)
            if verbose:
                print(f"Adding to explored. Current count: {len(explored)}")
                file_obj.write(f"Adding to explored. Current count: {len(explored)}\n")
            
            children = make_child_node(node, node.goal, board)
            if verbose:
                print(f"Generating children nodes..")
                file_obj.write(f"Generating children nodes..\n")
                for child in children:
                    print(f"-> Action: {child.action}, State: {child.state}")
                    file_obj.write(f"-> Action: {child.action}, State: {child.state}\n")
            
            for child in children:
                if verbose:
                    print(f"....Taking child: {child.state}")
                    file_obj.write(f"....Taking child: {child.state}\n")
                gen_nodes = gen_nodes + 1
                if child.str_state not in explored:
                    if verbose:
                        print(f"\tChild not explored. Adding to Frontier & Explored")
                        file_obj.write(f"\tChild not explored. Adding to Frontier & Explored\n")
                    frontier.append(child)
                    explored.add(child.str_state)
                    if len(frontier) > frontier_size:
                        frontier_size = len(frontier)
                    
                    max_depth = child.depth
                    if verbose:
                        print(f"\tCurrent Depth: {max_depth}")
                        file_obj.write(f"\tCurrent Depth: {max_depth}\n")
                    
                    if child.str_state == child.goal_str:
                        end_final = time.perf_counter() 
                        return Result(child, "success", frontier_size, child.depth, gen_nodes, end_final - start)     
                else:
                    if verbose:
                        print(f"\tChild explored. Skipping")
                        file_obj.write(f"\tChild explored. Skipping\n")
            if verbose:
                print(f"Nodes generated: {gen_nodes}")
                file_obj.write(f"Nodes generated: {gen_nodes}\n")
            
    end_failed = time.perf_counter()
    return Result(None, "failed", frontier_size, max_depth, gen_nodes, end_failed - start)
