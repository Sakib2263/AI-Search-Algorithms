from path_state import make_child_node, h_function
from collections import deque
from structures import Result
import time
import heapq

def gbfs(problem, board, verbose=True):
    start = time.perf_counter()
    
    max_depth = 0
    gen_nodes = 0
    
    problem.h_cost = h_function(problem)
    _node = (problem.h_cost, problem)
    frontier = [_node]
    frontier_size = 0
    explored = set()
    
    if problem.str_state == problem.goal_str:
        end_prem = time.perf_counter()
        return Result(problem, "success", frontier_size, max_depth, gen_nodes, end_prem - start)
   
    heapq.heapify(frontier)
   
    while frontier:
        _node = heapq.heappop(frontier)
        node = _node[1]
        #print(f"prio: {_node[0]} -> node: {node.state}")
        explored.add(node.str_state)
        
        children = make_child_node(node, node.goal, board)
        
        for child in children:
            gen_nodes = gen_nodes + 1
            if child.str_state not in explored:
                child.h_cost = h_function(child)
                _node = (child.h_cost, child)
                heapq.heappush(frontier, _node)
                explored.add(child.str_state)
                
                if max_depth < child.depth:
                    max_depth = child.depth
                
                if child.str_state == child.goal_str:
                    end_final = time.perf_counter()
                    return Result(child, "success", frontier_size, max_depth, gen_nodes, end_final - start)
        
        if len(frontier) > frontier_size:
            frontier_size = len(frontier)
            
        
    end_failed = time.perf_counter()
    return Result(None, "failed", frontier_size, max_depth, gen_nodes, end_failed - start)

