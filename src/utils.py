import pybullet as p
import numpy as np

def find_closest_cloth_node(cloth_id, position):
    """Finds the closest node on the cloth to a given position."""
    cloth_data = p.getMeshData(cloth_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    vertices = cloth_data[1]
    
    min_distance = float('inf')
    closest_node = None
    for i, vertex in enumerate(vertices):
        distance = np.linalg.norm(np.array(vertex) - np.array(position))
        if distance < min_distance:
            min_distance = distance
            closest_node = i
    print(f"Closest node for position {position} is node {closest_node} at distance {min_distance}")
    return closest_node
