import heapq
import math


NODE_COORDS = {
    0: (0.0, 0.0),
    1: (0.269, -0.049),
    2: (1.127, -1.075),
    3: (1.127, -0.820),
    4: (2.255, -0.049),
    5: (1.984, -0.049),
    6: (1.013, 1.081),
    7: (1.235, 0.791),
    8: (-0.749, 1.077),
    9: (-0.749, 0.790),
    10: (-1.282, -0.59),
    11: (0.0, 2.033),
    12: (0.0, 1.72),
    13: (0.269, 1.72),
    14: (2.255, 2.837),
    15: (1.984, 1.72),
    16: (0.908, 3.58),
    17: (1.466, 3.02),
    18: (0.623, 2.937),
    19: (0.792, 2.729),
    20: (0.0, 4.421),
    21: (0.0, 4.146),
    22: (-1.984, 2.837),
    23: (-1.716, 2.837),
    24: (-0.826, 3.645),
    25: (-0.857, 1.716),
}

PREDEFINED_RIDES = {
    "A": [1, 8], "B": [6, 18], "C": [2, 6], "D": [13, 6],
    "E": [1, 11], "F": [14, 11], "G": [8, 18], "H": [4, 0],
    "I": [14, 22], "J": [14, 24], "K": [24, 22], "L": [20, 7, 1],
    "M": [4, 1, 8], "N": [13, 3], "O": [24, 20], "P": [9, 15],
    "Q": [7, 23, 11], "R": [18, 5], "S": [15, 22], "T": [10, 0, 23],
    "U": [15, 7, 13], "V": [23, 14, 2], "W": [25, 8], "X": [24, 0, 25],
}

CROSSWALK_RECTS = [
    (-2.067, -1.52, 0.070, 0.314),
    (1.311, 1.591, 0.564, 1.25),
    (0.64, 0.87, 0.614, 1.3),
    (-0.209, 0.512, 1.455, 1.699),
    (-0.629, -0.358, 0.587, 1.265),
    (-0.25, 0.548, 0.2, 0.434),
]


class CityGraph:
    def __init__(self):
        self.edges = {
            0: [2],
            1: [7, 8, 13],
            2: [4],
            3: [1],
            4: [6, 14],
            5: [3],
            6: [0, 8, 13],
            7: [5, 14],
            8: [10, 23],
            9: [0, 7, 13],
            10: [1, 2],
            11: [12],
            12: [0, 7, 8],
            13: [19],
            14: [16, 20],
            15: [5, 6],
            16: [17, 18],
            17: [15, 16, 20],
            18: [11],
            19: [17],
            20: [22],
            21: [16],
            22: [9, 10],
            23: [21],
        }

    def _point_segment_distance(self, px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return math.hypot(px - x1, py - y1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

    def _is_on_right_side(self, px, py, x1, y1, x2, y2):
        return ((x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)) <= 0

    def get_closest_edge(self, x, y):
        best_edge = None
        min_score = float("inf")
        closest_node = min(
            NODE_COORDS.keys(),
            key=lambda n: math.hypot(x - NODE_COORDS[n][0], y - NODE_COORDS[n][1]),
        )

        for u, neighbors in self.edges.items():
            for v in neighbors:
                p1, p2 = NODE_COORDS.get(u), NODE_COORDS.get(v)
                dist = self._point_segment_distance(x, y, p1[0], p1[1], p2[0], p2[1])
                is_right = self._is_on_right_side(x, y, p1[0], p1[1], p2[0], p2[1])
                score = dist
                if u == closest_node or v == closest_node:
                    if is_right or dist < 0.15:
                        score -= 100.0
                if not is_right and dist > 0.15:
                    score += 100.0

                if score < min_score:
                    min_score, best_edge = score, (u, v)
        return best_edge if best_edge else (10, 1)

    def find_shortest_path(self, start_node, end_node):
        queue = [(0.0, start_node, [])]
        visited = set()
        min_cost = {node: float("inf") for node in self.edges}
        min_cost[start_node] = 0.0

        while queue:
            cost, current, path = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]

            if current == end_node:
                return path, cost

            for neighbor in self.edges.get(current, []):
                p1, p2 = NODE_COORDS.get(current), NODE_COORDS.get(neighbor)
                weight = math.hypot(p1[0] - p2[0], p1[1] - p2[1]) if p1 and p2 else 1.0
                if cost + weight < min_cost.get(neighbor, float("inf")):
                    min_cost[neighbor] = cost + weight
                    heapq.heappush(queue, (cost + weight, neighbor, path))

        return None, float("inf")


def is_point_in_crosswalk(point):
    x, y = float(point[0]), float(point[1])
    return any(
        min_x <= x <= max_x and min_y <= y <= max_y
        for min_x, max_x, min_y, max_y in CROSSWALK_RECTS
    )
