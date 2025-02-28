import heapq
from collections import deque

def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

def get_neighbors(point):
    neighbors = [
        (point[0] - 1, point[1]),  # up
        (point[0] + 1, point[1]),  # down
        (point[0], point[1] - 1),  # left
        (point[0], point[1] + 1)  # right
    ]
    return [n for n in neighbors if 0 <= n[0] < 5 and 0 <= n[1] < 5]

def a_star_search(start, goal, obstacles):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in get_neighbors(current):
            if neighbor in obstacles:
                continue
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return "No path"

start = (0, 0)
goal = (4, 4)
obstacles = [(1, 1), (2, 2), (3, 3)]
print(a_star_search(start, goal, obstacles))



print("##############################################")


def open_lock(deadends, target):
    def neighbors(node):
        for i in range(4):
            x = int(node[i])
            for d in (-1, 1):
                y = (x + d) % 10
                yield node[:i] + str(y) + node[i + 1:]

    dead = set(deadends)
    queue = deque([('0000', 0)])
    visited = {'0000'}

    while queue:
        node, depth = queue.popleft()
        if node == target:
            return depth
        if node in dead:
            continue
        for neighbor in neighbors(node):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, depth + 1))

    return -1


deadends = ["0201", "0101", "0102", "1212", "2002"]
target = "0202"
print(open_lock(deadends, target))



print("##############################################")


def uniform_cost_search(graph, start, goal):
    pq = [(0, start, [])] # priority queue to store cost node path
    visited = set()

    while pq:
        cost, node, path = heapq.heappop(pq)

        if node in visited:
            continue

        path = path + [node]
        visited.add(node)

        if node == goal:
            return path, cost

        for neighbor, weight in graph[node]:
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path))

    return "No path", float('inf')


graph = {  # graph as an adjacency list
    0: [(1, 2), (2, 4)],
    1: [(2, 1), (3, 7)],
    2: [(4, 3)],
    3: [(4, 1)],
    4: []
}

start = 0
goal = 4

path, cost = uniform_cost_search(graph, start, goal)
print(f"Path: {' -> '.join(map(str, path))}")
print(f"Cost: {cost}")


print("##############################################")




def shortest_bridge(grid):
    n = len(grid)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    def dfs(x, y, visited, island):
        stack = [(x, y)]
        while stack:
            cx, cy = stack.pop()
            if (cx, cy) not in visited:
                visited.add((cx, cy))
                island.append((cx, cy))
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < n and 0 <= ny < n and grid[nx][ny] == 1 and (nx, ny) not in visited:
                        stack.append((nx, ny))

    def bfs(queue, visited):
        steps = 0
        while queue:
            for _ in range(len(queue)):
                x, y = queue.popleft()
                for dx, dy in directions:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < n and 0 <= ny < n and (nx, ny) not in visited:
                        if grid[nx][ny] == 1:
                            return steps
                        queue.append((nx, ny))
                        visited.add((nx, ny))
            steps += 1
        return -1

    visited = set()
    island1 = []
    found = False

    for i in range(n):
        for j in range(n):
            if grid[i][j] == 1 and (i, j) not in visited:
                dfs(i, j, visited, island1)
                found = True
                break
        if found:
            break

    queue = deque(island1)
    visited = set(island1)

    return bfs(queue, visited)



grid1 = [[0, 1], [1, 0]]
grid2 = [[0, 1, 0], [0, 0, 0], [0, 0, 1]]

print(shortest_bridge(grid1))
print(shortest_bridge(grid2))