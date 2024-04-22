import heapq

# Dijkstra's Algorithm
def dijkstra(graph, start, goal):
    queue = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        
        if current_node == goal:
            break
        
        if current_distance > distances[current_node]:
            continue
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))
    
    return distances[goal]

# A* Algorithm
def heuristic(node, goal):
    # Define heuristic function here (e.g., Manhattan distance)
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def astar(graph, start, goal):
    queue = [(0, start)]
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    
    while queue:
        current_score, current_node = heapq.heappop(queue)
        
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1]
        
        for neighbor, weight in graph[current_node].items():
            tentative_g_score = g_score[current_node] + weight
            if tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(queue, (f_score, neighbor))
                came_from[neighbor] = current_node

# Uniform Cost Search
def uniform_cost_search(graph, start, goal):
    queue = [(0, start)]
    came_from = {}
    cost_so_far = {node: float('inf') for node in graph}
    cost_so_far[start] = 0
    
    while queue:
        _, current_node = heapq.heappop(queue)
        
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1]
        
        for neighbor, weight in graph[current_node].items():
            new_cost = cost_so_far[current_node] + weight
            if new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heapq.heappush(queue, (new_cost, neighbor))
                came_from[neighbor] = current_node

# Bidirectional Search
def bidirectional_search(graph, start, goal):
    forward_queue = [(0, start)]
    backward_queue = [(0, goal)]
    forward_visited = {start: 0}
    backward_visited = {goal: 0}
    
    while forward_queue and backward_queue:
        forward_distance, forward_node = heapq.heappop(forward_queue)
        backward_distance, backward_node = heapq.heappop(backward_queue)
        
        if forward_node in backward_visited:
            path = [forward_node]
            current_node = forward_node
            while current_node != start:
                current_node = forward_visited[current_node]
                path.append(current_node)
            current_node = backward_node
            while current_node != goal:
                current_node = backward_visited[current_node]
                path.append(current_node)
            return path[::-1]
        
        if backward_node in forward_visited:
            path = [backward_node]
            current_node = backward_node
            while current_node != goal:
                current_node = backward_visited[current_node]
                path.append(current_node)
            current_node = forward_node
            while current_node != start:
                current_node = forward_visited[current_node]
                path.append(current_node)
            return path
        
        for neighbor, weight in graph[forward_node].items():
            distance = forward_distance + weight
            if neighbor not in forward_visited or distance < forward_visited[neighbor]:
                forward_visited[neighbor] = distance
                heapq.heappush(forward_queue, (distance, neighbor))
                
        for neighbor, weight in graph[backward_node].items():
            distance = backward_distance + weight
            if neighbor not in backward_visited or distance < backward_visited[neighbor]:
                backward_visited[neighbor] = distance
                heapq.heappush(backward_queue, (distance, neighbor))

# Bellman-Ford Algorithm
def bellman_ford(graph, start, goal):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    for _ in range(len(graph) - 1):
        for node in graph:
            for neighbor, weight in graph[node].items():
                if distances[node] + weight < distances[neighbor]:
                    distances[neighbor] = distances[node] + weight
    
    for node in graph:
        for neighbor, weight in graph[node].items():
            if distances[node] + weight < distances[neighbor]:
                return "Negative cycle detected"
    
    path = []
    current_node = goal
    while current_node != start:
        path.append(current_node)
        for neighbor, weight in graph[current_node].items():
            if distances[current_node] == distances[neighbor] + weight:
                current_node = neighbor
                break
    path.append(start)
    return path[::-1]
