import pybullet as p
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

def initialize_network_visualization():
    """Sets up the Matplotlib figure and axis for visualizing robot network."""
    fig, ax = plt.subplots()
    ax.set_title("Robot Network - Top View")
    ax.axis("equal")
    return fig, ax

def update_network_visualization(ax, robots, state_data, neighbor_distance, perception_range, max_stretch, closest_floor_distance, collision_info):
    """Updates the network visualization based on robot states, distances, cloth metrics, and collisions."""
    ax.clear()  # Clear the previous frame
    G = nx.Graph()

    # Add nodes for each robot with positions
    pos = {}
    for robot_id, state in state_data.items():
        G.add_node(robot_id)
        pos[robot_id] = (state['position'][0], state['position'][1])

    # Add edges for neighbors (green) and obstacles (red)
    for robot_id, state in state_data.items():
        # Check for neighbors within neighbor_distance
        for neighbor_position in state['neighbors']:
            neighbor_id = next((id for id, s in state_data.items() if s['position'] == neighbor_position), None)
            if neighbor_id:
                G.add_edge(robot_id, neighbor_id, color='green', weight=1)

        # Check for obstacles within perception range
        for obstacle_position in state['nearby_obstacles']:
            obstacle_id = f"obstacle_{obstacle_position}"
            if obstacle_id not in G:
                G.add_node(obstacle_id)  # Add obstacle node
                pos[obstacle_id] = (obstacle_position[0], obstacle_position[1])  # Set its position
            G.add_edge(robot_id, obstacle_id, color='red', weight=0.5)

    # Draw the graph with color-coded edges
    edge_colors = [G[u][v]['color'] for u, v in G.edges()]
    nx.draw(G, pos, ax=ax, edge_color=edge_colors, with_labels=True, node_color="skyblue", node_size=300)

    # Display cloth metrics
    ax.text(0.05, 0.95, f"Max Cloth Stretch: {max_stretch:.2f}", transform=ax.transAxes, fontsize=12, color='blue', verticalalignment='top')
    ax.text(0.05, 0.90, f"Closest Cloth to Floor: {closest_floor_distance:.2f}", transform=ax.transAxes, fontsize=12, color='purple', verticalalignment='top')

    # Display collision information
    if collision_info:
        collision_text = "Collisions: " + ", ".join([f"Robot {r} with Obstacle {o}" for r, o in collision_info])
        ax.text(0.05, 0.85, collision_text, transform=ax.transAxes, fontsize=12, color='red', verticalalignment='top')

    # Refresh the Matplotlib canvas
    plt.pause(0.01)



def get_robot_state(robot_id, robots, cloth_id, obstacle_ids, perception_range, neighbor_distance, initial_distances, collision_radius):
    """Extracts the state for a given robot, including local obstacles, neighbors, estimated cloth stretch, ground contact, and collision detection."""
    
    # Get robot's current position and velocity
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)
    
    # Detect nearby obstacles within perception range
    nearby_obstacles = []
    collision_flag = None  # Stores (robot_id, obstacle_id) if a collision occurs
    for obstacle_id in obstacle_ids:
        obstacle_position, _ = p.getBasePositionAndOrientation(obstacle_id)
        
        # Calculate 2D distance (ignore z-coordinate)
        distance = np.linalg.norm(np.array(position[:2]) - np.array(obstacle_position[:2]))
        
        if distance <= perception_range:
            nearby_obstacles.append(obstacle_position)
        # Check for collision based on 2D distance and collision radius
        if distance <= collision_radius:
            collision_flag = (robot_id, obstacle_id)

    # Detect neighboring robots within neighbor distance and estimate cloth stretch
    neighbors = []
    estimated_cloth_stretch = 0
    num_neighbors = 0
    for other_robot_id in robots:
        if other_robot_id != robot_id:
            other_position, _ = p.getBasePositionAndOrientation(other_robot_id)
            
            # Calculate 2D distance for neighbors
            distance = np.linalg.norm(np.array(position[:2]) - np.array(other_position[:2]))
            
            if distance <= neighbor_distance:
                neighbors.append(other_position)
                # Estimate stretch based on initial distance
                initial_distance = initial_distances[(robot_id, other_robot_id)]
                estimated_cloth_stretch += max(0, distance - initial_distance)
                num_neighbors += 1

    # Average the estimated stretch across neighbors
    estimated_cloth_stretch = estimated_cloth_stretch / num_neighbors if num_neighbors > 0 else 0

    # Check if the cloth node is close to the ground (assume cloth node index = robot_id)
    cloth_data = p.getMeshData(cloth_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    vertices = cloth_data[1]
    attached_node_position = vertices[robot_id]
    is_cloth_near_ground = attached_node_position[2] <= 0.05  # Threshold for ground contact

    # Construct the state dictionary
    state = {
        'position': position,
        'velocity': linear_velocity,
        'nearby_obstacles': nearby_obstacles,
        'neighbors': neighbors,
        'estimated_cloth_stretch': estimated_cloth_stretch,
        'is_cloth_near_ground': is_cloth_near_ground,
        'collision_flag': collision_flag  # Store collision info
    }

    return state


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
