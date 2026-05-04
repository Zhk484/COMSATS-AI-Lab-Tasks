# Import the heapq module, which provides an implementation of the heap queue algorithm.
# This will be used as our priority queue to efficiently get the node with the lowest f-score.
import heapq

# Define a class to represent our Graph structure
class Graph:
    # Initialize the Graph object when it is created
    def __init__(self):
        # Create a dictionary to store the adjacency list representing our graph's edges.
        # The structure will be: { 'node_name': { 'neighbor_name': edge_weight } }
        self.edges = {}
        
        # Create a dictionary to store the heuristic values for each node.
        # The structure will be: { 'node_name': heuristic_value }
        # Heuristics represent the estimated cost from a node to the goal.
        self.heuristics = {}

    # Define a method to add a new node and its heuristic value to the graph
    def add_node(self, name, heuristic_val):
        # Store the provided heuristic value into our heuristics dictionary using the node's name as the key
        self.heuristics[name] = heuristic_val
        
        # Check if this node is already present in our edges dictionary
        if name not in self.edges:
            # If it's not present, initialize an empty dictionary to hold its future neighbors
            self.edges[name] = {}

    # Define a method to add a weighted edge between two nodes (u and v)
    def add_edge(self, u, v, weight):
        # Ensure node 'u' exists as a key in the edges dictionary
        if u not in self.edges:
            self.edges[u] = {}
            
        # Ensure node 'v' exists as a key in the edges dictionary
        if v not in self.edges:
            self.edges[v] = {}
            
        # Record 'v' as a neighbor of 'u', assigning the travel cost (weight) between them
        self.edges[u][v] = weight
        
        # Record 'u' as a neighbor of 'v' with the same weight.
        # Including this line makes the graph undirected (you can travel both ways).
        # Removing this line would make the graph directed (one-way).
        self.edges[v][u] = weight 

    # Define the A* search algorithm method, which takes a starting node and a target goal node
    def a_star_search(self, start, goal):
        # Initialize open_set as an empty list. This will act as our priority queue.
        # The open set contains nodes that need to be evaluated.
        open_set = []
        
        # Push the starting node onto the priority queue. 
        # Elements are stored as tuples: (f_score, node_name). 
        # We start with f_score = the heuristic of the start node (since cost so far is 0).
        heapq.heappush(open_set, (self.heuristics.get(start, 0), start))
        
        # Initialize a dictionary to keep track of the most efficient previous node for each visited node.
        # This acts like breadcrumbs so we can reconstruct the final optimal path later.
        came_from = {}
        
        # Initialize the g_score dictionary. g_score is the exact, known cost to reach a node from the start.
        # Initially, set the cost to reach all known nodes to infinity.
        g_score = {node: float('inf') for node in self.heuristics}
        
        # The exact cost to reach the start node from itself is exactly 0.
        g_score[start] = 0
        
        # Initialize the f_score dictionary. f_score is the total estimated cost of a path passing through a node.
        # Equation: f_score(n) = g_score(n) + heuristic(n)
        # Initially, set the f_score for all known nodes to infinity.
        f_score = {node: float('inf') for node in self.heuristics}
        
        # For the start node, the f_score is purely its heuristic value (since its g_score is 0).
        f_score[start] = self.heuristics.get(start, 0)
        
        # Begin the main loop: continue searching as long as there are nodes to evaluate in our open_set
        while open_set:
            # Pop the node with the lowest f_score from the priority queue.
            # current_f is that lowest score, and 'current' is the actual node name.
            current_f, current = heapq.heappop(open_set)
            
            # Check if the node we just popped is our target goal node
            if current == goal:
                # If we've reached the goal, we are done! Initialize a list to hold the final path.
                path = []
                
                # Trace our steps backward from the goal node using our 'came_from' breadcrumbs
                while current in came_from:
                    # Append the current node to our path
                    path.append(current)
                    # Shift our attention to the node that led us to this current one
                    current = came_from[current]
                    
                # Finally, append the starting node to complete the trace
                path.append(start)
                
                # Since we traced backward (Goal -> Start), we must reverse the list to get (Start -> Goal)
                path.reverse()
                
                # Return the completed, optimal path
                return path
                
            # If we haven't reached the goal, we need to evaluate all neighbors of the 'current' node.
            # We use .get(current, {}) to safely return an empty dictionary if the node has no outgoing edges.
            for neighbor, weight in self.edges.get(current, {}).items():
                
                # Calculate a 'tentative' g_score for the neighbor.
                # This is the cost to reach 'current' PLUS the weight of the edge to the 'neighbor'.
                tentative_g_score = g_score[current] + weight
                
                # Check if this newly calculated cost is cheaper than any previously known cost to reach the neighbor
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # If it IS cheaper, we have found a better route!
                    # Record that the best way to reach 'neighbor' is by coming from 'current'
                    came_from[neighbor] = current
                    
                    # Update the known g_score for this neighbor to the new, cheaper cost
                    g_score[neighbor] = tentative_g_score
                    
                    # Update the f_score for this neighbor (new g_score + the neighbor's heuristic value)
                    f_score[neighbor] = tentative_g_score + self.heuristics.get(neighbor, 0)
                    
                    # Now we must check if the neighbor is already waiting in the open_set (priority queue)
                    # We iterate through the open_set to check if the neighbor's name matches any stored node
                    if not any(neighbor == item[1] for item in open_set):
                        # If it is NOT in the queue, push it in so it will be evaluated in a future loop iteration
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        # If the while loop finishes and open_set becomes empty, it means we explored everything
        # and never reached the goal node. Therefore, no path exists.
        return None 


if __name__ == "__main__":
    # Instantiate a new Graph object
    g = Graph()

    # Add nodes to the graph along with their heuristic values (estimated cost to reach 'Goal')
    g.add_node('Start', 7)
    g.add_node('A', 6)
    g.add_node('B', 2)
    g.add_node('C', 1)
    g.add_node('Goal', 0)

    # Add edges connecting the nodes along with their actual travel costs (weights)
    g.add_edge('Start', 'A', 1)
    g.add_edge('Start', 'B', 4)
    g.add_edge('A', 'B', 2)
    g.add_edge('A', 'C', 5)
    g.add_edge('B', 'C', 2)
    g.add_edge('C', 'Goal', 3)

    # Print out the nodes and their heuristics to verify data
    print("Nodes and Heuristics:")
    for node, h in g.heuristics.items():
        print(f"Node: {node}, Heuristic: {h}")
        
    # Print out the edges and their weights to verify data
    print("\nEdges and Weights (Undirected):")
    printed_edges = set() # Use a set to avoid printing undirected edges twice
    for node, neighbors in g.edges.items():
        for neighbor, weight in neighbors.items():
            edge_tuple = tuple(sorted([node, neighbor]))
            if edge_tuple not in printed_edges:
                print(f"{node} <--> {neighbor} (Weight: {weight})")
                printed_edges.add(edge_tuple)

    # Execute the A* search from 'Start' to 'Goal'
    print("\nFinding path from 'Start' to 'Goal' using A* Search...")
    path = g.a_star_search('Start', 'Goal')
    
    # Check if a valid path was returned and print the result
    if path:
        print(f"Optimal Path: {' -> '.join(path)}")
    else:
        print("No path found.")
