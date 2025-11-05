from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot

INF = 1e10

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    from collections import deque
    queue = deque()
    queue.append(start)
    visited = {start}
    came_from = {}

    while queue:

        current = queue.popleft()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    stack = []
    stack.append(start)
    visited = {start}
    came_from = {}

    while stack:

        current = stack.pop()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Euclidian distance between p1 and p2.
    """
    pass


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))
    came_from = {}
    g_score = {}
    for row in grid.grid:
        for spot in row:
            g_score[spot] = INF
    g_score[start] = 0
    f_score = {}
    for row in grid.grid:
        for spot in row:
            f_score[spot] = INF
    f_score[start] = h_manhattan_distance(start.get_position(), end.get_position())
    lookup_set = {start}

    while open_heap:

        current = open_heap.get()[2]
        lookup_set.remove(current)

        if current is end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            start.make_start()
            end.make_end()
            return True
        
        for neighbor in current.neighbors:
            tentative_g = g_score[current] + 1
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h_manhattan_distance(neighbor.get_position(), end.get_position())
                if neighbor not in lookup_set:
                    count += 1
                    open_heap.put((f_score[neighbor], count, neighbor))
                    lookup_set.add(neighbor)
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

def dls(draw: callable, grid: Grid, start: Spot, end: Spot, limit: int) -> bool:
    """
    Depdth-Limited Search (DLS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    stack = []
    stack.append((start, 0))
    visited = {start}
    came_from = {}

    while stack:

        (current, depth) = stack.pop()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier() and depth + 1 <= limit:
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append((neighbor, depth + 1))
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False

def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Uniform-Cost Search (UCS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    import heapq
    queue = [(0, start)]
    heapq.heapify(queue)
    visited = {start: 0}
    came_from = {}

    while queue:

        d, current = heapq.heappop(queue)
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if (neighbor not in visited or d + 1 < visited[neighbor]) and not neighbor.is_barrier():
                visited[neighbor] = d + 1
                came_from[neighbor] = current
                heapq.heappush(queue, (d + 1, neighbor))
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()
    return False

def dijkstra(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Dijkstra Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    import heapq
    queue = [(0, start)]
    heapq.heapify(queue)
    visited = {start: 0}
    came_from = {}

    while queue:

        d, current = heapq.heappop(queue)
            
        for neighbor in current.neighbors:
            if (neighbor not in visited or d + 1 < visited[neighbor]) and not neighbor.is_barrier():
                visited[neighbor] = d + 1
                came_from[neighbor] = current
                heapq.heappush(queue, (d + 1, neighbor))
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    if end in visited:
        current = end
        while current in came_from:
            current = came_from[current]
            current.make_path()
            draw()
        end.make_end()
        start.make_start()
        return True
    return False

def iddfs(draw: callable, grid: Grid, start: Spot, end: Spot, max_depth: int) -> bool:
    """
    Iterative Deepening Depth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    for limit in range(1, max_depth + 1):
        if dls(draw, grid, start, end, limit) == True:
            return True
    return False

def astar_bounded(draw: callable, grid: Grid, path: list[Spot], end: Spot, g: float, bound: float) -> bool:
    
    current = path[len(path) - 1]
    f = g + h_manhattan_distance(current.get_position(), end.get_position())
    if f > bound: 
        return f
    if current == end:
        return -1
    min = INF

    if current != path[0]:
        current.make_closed()

    
    for neighbour in sorted(current.neighbors, key = lambda n: g + h_manhattan_distance(n.get_position(), end.get_position())):
        if neighbour not in path:
            neighbour.make_open()
            path.append(neighbour)
            t = astar_bounded(draw, grid, path, end, g + 1, bound)
            if t == -1:
                return -1
            if t < min:
                min = t
            path.pop()
    
    
    
    draw()

    return min 

def idastar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Iterative Deepening A* (IDA*) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    bound = h_manhattan_distance(start.get_position(), end.get_position())
    path = [start]
    while True:
        t = astar_bounded(draw, grid, path, end, 0, bound)
        if t == -1:
            for p in path:
                if not p.is_start() and not p.is_end():
                    p.make_path()
                    draw()
            return True
        if t == INF:
            return False
        bound = t



# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equals 1