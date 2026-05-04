import heapq

def uniform_cost_search(graph, start, goal):
   
    # Priority queue stores tuples of (cost_so_far, node)
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    # Keep track of the best previous node to reconstruct the path
    came_from = {}
    
    # Keep track of the exact minimum cost to reach each node
    g_score = {start: 0}
    
    # Keep track of fully explored nodes to avoid redundant work
    explored = set()
    
    while open_set:
        # Pop the node with the lowest cumulative cost
        current_cost, current = heapq.heappop(open_set)
        
        # Check if we've reached the goal
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, current_cost
            
        # If the node has already been explored with a cheaper or equal cost, skip it
        if current in explored:
            continue
            
        # Mark the current node as explored
        explored.add(current)
        
        # Evaluate all neighbors of the current node
        for neighbor, weight in graph.get(current, {}).items():
            tentative_g_score = current_cost + weight
            
            # If we found a cheaper path to the neighbor, update it
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                # Push the neighbor into the priority queue with its new cost
                heapq.heappush(open_set, (tentative_g_score, neighbor))
                
    # If the queue is empty and goal hasn't been reached, return no path
    return None, float('inf')


if __name__ == "__main__":
    # Define the graph using a simple dictionary of dictionaries.
    # The keys are nodes, and the values are dictionaries representing connected neighbors and the edge weights.
    # This represents an undirected graph, so connections go both ways with the same weight.
    graph = {
        'Start': {'A': 1, 'B': 4},
        'A': {'Start': 1, 'B': 2, 'C': 5},
        'B': {'Start': 4, 'A': 2, 'C': 2},
        'C': {'A': 5, 'B': 2, 'Goal': 3},
        'Goal': {'C': 3}
    }

    # Print out the edges and their weights to verify data
    print("Edges and Weights (Undirected):")
    printed_edges = set() 
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            edge_tuple = tuple(sorted([node, neighbor]))
            if edge_tuple not in printed_edges:
                print(f"{node} <--> {neighbor} (Weight: {weight})")
                printed_edges.add(edge_tuple)

    # Execute the Uniform Cost Search from 'Start' to 'Goal'
    print("\nFinding path from 'Start' to 'Goal' using Uniform Cost Search...")
    path, cost = uniform_cost_search(graph, 'Start', 'Goal')
    
    # Check if a valid path was returned and print the result
    if path:
        print(f"UCS Optimal Path: {' -> '.join(path)} (Total Cost: {cost})")
    else:
        print("No path found.")
