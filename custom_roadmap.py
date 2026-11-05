import numpy as np
from hal.utilities.path_planning import RoadMap
#file changed
# ==========================================
# SCALING FACTOR - single source of truth
# Node positions and non-zero radii are multiplied by this factor.
# ==========================================
SCALE_FACTOR = 1.0

# ==========================================
# TRANSLATION OFFSET - move the roadmap without changing its shape
# ==========================================
X_OFFSET = 0
Y_OFFSET = -0

# ==========================================
# CALIBRATED NODE COORDINATES (from new code)
# ==========================================
NODE_DATA_BASE = {
    0: (0.0405, 0.1974, -90),
    1: (0.3345, -0.2573, 93),
    2: (0.8492, -1.1027, 0),
    3: (1.3674, -0.8521, -180),
    4: (2.374, -0.2420, 93),
    5: (1.9796, 0.2046, -90),
    6: (1.159, 1.18, -180),
    7: (1.23, 0.7998, 0),
    8: (-0.6949, 1.0688, -180),
    9: (-1.1199, 0.7714, 0),
    10: (-1.564, -0.2578, -40),
    11: (-0.0198, 2.1524, -90),
    12: (-0.05, 1.8546, -90),
    13: (0.2825, 1.7843, 90),
    14: (2.28, 2.473, 90),
    15: (1.9550, 2.1168, -90),
    16: (0.85, 3.58, -80.6),
    17: (1.2648, 3.1542, -9.4),
    18: (0.6097, 3.0936, -135),
    19: (0.5738, 2.6124, 48),
    20: (0.2643, 4.485, -180),
    21: (-0.3106, 4.1664, 0),
    22: (-2.02, 2.9721, -90),
    23: (-1.7020, 2.7449, 90),
}


class CustomRoadMap(RoadMap):
    """
    Hybrid RoadMap: Calibrated node positions + proper road geometry radii.
    """

    def __init__(self, scale_factor=None, x_offset=None, y_offset=None):
        super().__init__()
        if scale_factor is None:
            scale_factor = SCALE_FACTOR
        if x_offset is None:
            x_offset = X_OFFSET
        if y_offset is None:
            y_offset = Y_OFFSET

        self.scale_factor = float(scale_factor)
        self.x_offset = float(x_offset)
        self.y_offset = float(y_offset)

        node_data = {
            node_id: (
                x * self.scale_factor + self.x_offset,
                y * self.scale_factor + self.y_offset,
                heading,
            )
            for node_id, (x, y, heading) in NODE_DATA_BASE.items()
        }

        # Original SDCS radii from mats.py
        scale = 0.002035
        innerLaneRadius = 310 * scale  # 0.621
        outerLaneRadius = 424 * scale  # 0.891
        trafficCircleRadius = 180 * scale  # 0.677
        oneWayStreetRadius = 350 * scale  # 0.712
        kinkStreetRadius = 375 * scale  # 0.763

        # print(
        #     f"[CustomRoadMap] Radii: inner={innerLaneRadius:.4f}, outer={outerLaneRadius:.4f}, circle={trafficCircleRadius:.4f}"
        # )

        # Edge configurations with radius values (from old code)
        # Format: [from_node, to_node, turn_radius]
        # Radii are scaled by the selected factor exactly once.
        edgeConfigs_base = [
            [0, 2, 0.7],
            [1, 7, innerLaneRadius],
            [1, 8, outerLaneRadius],
            [2, 4, 0.675],
            [3, 1, 0.55],
            [4, 6, outerLaneRadius],
            [5, 3, 0.55],
            [6, 0, .85],
            [6, 8, 0],
            [7, 5, 0.55],
            [8, 10, oneWayStreetRadius],
            [9, 0, 0.55],
            [9, 7, 0],
            [10, 1, 0.695],
            [10, 2, innerLaneRadius],
            [1, 13, 0],
            [4, 14, 0],
            [6, 13, .455],
            [7, 14, outerLaneRadius],
            [8, 23, innerLaneRadius],
            [9, 13, .85],
            [11, 12, 0],
            [12, 0, 0],
            [12, 7, .725],
            [12, 8, innerLaneRadius],
            [13, 19, innerLaneRadius],
            [14, 16, .73],  # Straight-line: 90Ã‚Â° to -80.6Ã‚Â° heading diff
            [14, 20, trafficCircleRadius],
            [15, 5, outerLaneRadius],
            [15, 6, .581],
            [16, 17, 0.32],
            [16, 18, 0.4],
            [17, 15, innerLaneRadius],
            [17, 16, 0.665],
            [17, 20, .695],  # Straight-line: -9.4Ã‚Â° to -180Ã‚Â° heading diff
            [18, 11, 0.55],
            [19, 17, 0.25],  # UPDATED to 0.35 (was 0.5)
            [20, 22, outerLaneRadius],
            [21, 16, 0.35],  # UPDATED to 0.35 (was 0.395)
            [22, 9, 0.75],
            [22, 10, outerLaneRadius],
            [23, 21, 0.55],
        ]

        # Scale fixed radii (keep 0 as 0)
        edgeConfigs = [
            [from_node, to_node, radius * self.scale_factor if radius > 0 else 0]
            for from_node, to_node, radius in edgeConfigs_base
        ]

        # Add nodes with calibrated positions
        sorted_ids = sorted(node_data.keys())
        for node_id in sorted_ids:
            x, y, heading_deg = node_data[node_id]
            heading_rad = np.radians(heading_deg)
            self.add_node([x, y, heading_rad])

        # Add edges with proper radius values
        # The RoadMap class will use SCSPath to generate proper curved paths
        for edgeConfig in edgeConfigs:
            self.add_edge(*edgeConfig)

        # Check for failures but DO NOT fill with straight lines
        failed_edges = []
        for edge in self.edges:
            if edge.waypoints is None:
                from_id = self.nodes.index(edge.fromNode)
                to_id = self.nodes.index(edge.toNode)
                dist = np.linalg.norm(
                    edge.toNode.pose[:2, :] - edge.fromNode.pose[:2, :]
                )
                failed_edges.append((from_id, to_id, dist))

        # if len(failed_edges) > 0:
        #     print(
        #         f"\n[CustomRoadMap] CRITICAL: {len(failed_edges)} edges failed SCSPath generation!"
        #     )
        #     print("These edges have NO waypoints (straight-line fallback removed):")
        #     for from_id, to_id, dist in failed_edges:
        #         print(f"  Edge {from_id} -> {to_id} (distance={dist:.4f}m)")


if __name__ == "__main__":
    # Test the hybrid roadmap
    roadmap = CustomRoadMap()
    # print(
    #     f"Created roadmap with {len(roadmap.nodes)} nodes and {len(roadmap.edges)} edges"
    # )

    # Print some node information
    # for i in range(min(5, len(roadmap.nodes))):
    #     pose = roadmap.nodes[i].pose
    #     print(f"Node {i}: ({pose[0, 0]:.3f}, {pose[1, 0]:.3f}, {pose[2, 0]:.3f})")