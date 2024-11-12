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

def measure_max_stretch(cloth_id, initial_positions):
    """Measures the maximum stretching between cloth nodes relative to initial positions."""
    cloth_data = p.getMeshData(cloth_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    vertices = cloth_data[1]
    
    max_stretch = 0
    for i, vertex in enumerate(vertices):
        initial_position = np.array(initial_positions[i])
        current_position = np.array(vertex)
        stretch = np.linalg.norm(current_position - initial_position)
        if stretch > max_stretch:
            max_stretch = stretch

    return max_stretch

def is_cloth_touching_ground(cloth_id, ground_level=0.0, tolerance=0.1):
    """Checks if any cloth vertices are touching the ground within a tolerance."""
    cloth_data = p.getMeshData(cloth_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    vertices = cloth_data[1]
    
    for vertex in vertices:
        if vertex[2] <= ground_level + tolerance:
            return True  # Cloth is touching the ground

    return False  # No vertices are touching the ground
