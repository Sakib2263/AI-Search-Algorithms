from path_state import make_child_node, h_function
from collections import deque
from structures import Result
import time
import heapq

def ucs(problem, board, verbose=False):
    start = time.perf_counter()
    
    with open("log/ucs.txt", "w") as file_obj:
    
        gen_nodes = 0
        max_depth = 0
        
        _node = (problem.path_cost, problem)
        frontier = [_node]
        frontier_size = len(frontier)
        explored = set()
        mapp = {}
        
        if problem.str_state == problem.goal_str:
            end_prem = time.perf_counter()
            return Result(problem, "success", frontier_size, max_depth, gen_nodes, end_prem - start)
        
        heapq.heapify(frontier)
        
        while frontier:
            if verbose:
                print(f"Frontier (PriorityQ) before extraction: {len(frontier)}")
                file_obj.write(f"Frontier (PriorityQ) before extraction: {len(frontier)}\n")
            _node = heapq.heappop(frontier)
            if verbose:
                print(f"Frontier (PriorityQ) after extraction: {len(frontier)}")
                file_obj.write(f"Frontier (PriorityQ) after extraction: {len(frontier)}\n")
            node = _node[1]
            explored.add(node.str_state)
            if verbose:
                print(f"Adding to explored. Current count: {len(explored)}")
                file_obj.write(f"Adding to explored. Current count: {len(explored)}\n")
            
            if node.str_state == node.goal_str:
                end_final = time.perf_counter()
                return Result(node, "success", frontier_size, max_depth, gen_nodes, end_final - start)
            
            children = make_child_node(node, node.goal, board)
            if verbose:
                print(f"Generating children nodes..")
                file_obj.write(f"Generating children nodes..\n")
                for child in children:
                    print(f"-> Action: {child.action}, State: {child.state}")
                    file_obj.write(f"-> Action: {child.action}, State: {child.state}\n")
            for child in children:
                gen_nodes = gen_nodes + 1
                _node = (child.path_cost, child)
                if verbose:
                    print(f"....Taking child: {child.state}")
                    file_obj.write(f"....Taking child: {child.state}\n")
                if child.str_state not in explored:
                    if verbose:
                        print(f"    Child not explored. Adding to Frontier & Explored")
                        file_obj.write(f"    Child not explored. Adding to Frontier & Explored\n")
                    heapq.heappush(frontier, _node)
                    explored.add(child.str_state)
                    if verbose:
                        print(f"\tAdding child to Map: '{child.str_state}' -> {child}'")
                        file_obj.write(f"\tAdding child to Map: '{child.str_state}' -> {child}\n")
                    mapp[child.str_state] = child
                    
                    if max_depth < child.depth:
                        max_depth = child.depth
                        if verbose:
                            print(f"\tCurrent Depth: {max_depth}")
                            file_obj.write(f"\tCurrent Depth: {max_depth}\n")
                
                elif child.str_state in mapp and child.path_cost < mapp[child.str_state].path_cost:
                    if verbose:
                        print(f"\tBetter than child found in Map\n-> Updating ('{child.str_state}'-> {mapp[child.str_state]}, Cost: {mapp[child.str_state].path_cost} with ('{child.str_state}'-> {child}, Cost: {child.path_cost}))")
                    index = frontier.index((mapp[child.str_state].path_cost, mapp[child.str_state]))
                    frontier[int(index)] = (child.path_cost, child)
                    heapq.heapify(frontier)
                    mapp[child.str_state] = child
                else:
                    if verbose:
                        print(f"\tChild explored. Not a better child. Skipping")
                        file_obj.write("\tChild explored. Not a better child. Skipping\n")
            if verbose:
                print(f"Nodes generated: {gen_nodes}")
                file_obj.write(f"Nodes generated: {gen_nodes}\n")
            
            if len(frontier) > frontier_size:
                frontier_size = len(frontier)

    end_failed = time.perf_counter()
    return Result(None, "failed", frontier_size, max_depth, gen_nodes, end_failed - start)
