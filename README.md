# Minor-Project

#Common Code
from queue import PriorityQueue

# Define the graph as an adjacency list
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}
#Dijkstra's Algorithm
def dijkstra(graph, start, goal):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    visited = set()
    pq = PriorityQueue()
    pq.put((0, start))
    path = {}

    while not pq.empty():
        current_distance, current_node = pq.get()

        if current_node in visited:
            continue
        visited.add(current_node)

        # Stop if we reach the goal
        if current_node == goal:
            break

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                pq.put((distance, neighbor))
                path[neighbor] = current_node

    return reconstruct_path(path, start, goal), distances[goal]

def reconstruct_path(path, start, goal):
    node = goal
    result = []
    while node != start:
        result.append(node)
        node = path.get(node, start)
    result.append(start)
    return result[::-1]

# Example
shortest_path, cost = dijkstra(graph, 'A', 'D')
print("Dijkstra's Path:", shortest_path, "Cost:", cost)

# A* Algorithm

def heuristic(node, goal):
    # A simple heuristic (Manhattan or straight-line distance can be used in grids)
    heuristics = {'A': 4, 'B': 2, 'C': 1, 'D': 0}  # Example heuristic values
    return heuristics[node]

def a_star(graph, start, goal):
    pq = PriorityQueue()
    pq.put((0, start))
    g_scores = {node: float('infinity') for node in graph}
    g_scores[start] = 0
    path = {}

    while not pq.empty():
        current_f_score, current_node = pq.get()

        if current_node == goal:
            break

        for neighbor, weight in graph[current_node].items():
            tentative_g_score = g_scores[current_node] + weight
            f_score = tentative_g_score + heuristic(neighbor, goal)
            if tentative_g_score < g_scores[neighbor]:
                g_scores[neighbor] = tentative_g_score
                pq.put((f_score, neighbor))
                path[neighbor] = current_node

    return reconstruct_path(path, start, goal), g_scores[goal]

# Example
shortest_path, cost = a_star(graph, 'A', 'D')
print("A* Path:", shortest_path, "Cost:", cost)
