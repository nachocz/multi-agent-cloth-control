import pybullet as p
import pybullet_data
import time
from environment import initialize_environment, add_ground_plane, add_obstacles
from robot import create_robots, control_robot
from cloth import create_cloth, anchor_cloth_nodes
from utils import find_closest_cloth_node, measure_max_stretch, is_cloth_touching_ground

# Initialize PyBullet environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
initialize_environment()

# Load ground plane
plane_id = add_ground_plane()

# Define robot height and grasp height above robot
robot_height = 0.1
grasp_height_above_robot = 0.5
radius = 0.55  # Distance from cloth center to position robots near corners

# Load robots and helper objects
N = 4
robots, helper_objects, visual_spheres, velocities = create_robots(N, robot_height, grasp_height_above_robot, radius)

# Calculate the anchor height as the robot height plus the grasp height above the robot
anchor_height = robot_height + grasp_height_above_robot
print(f"Calculated anchor height based on robot and grasp heights: {anchor_height}")

# Load cloth just above the anchor height
cloth_id = create_cloth(anchor_height)

# Anchor cloth nodes to helper objects
anchor_cloth_nodes(cloth_id, helper_objects)

# Add obstacles
obstacles = add_obstacles(shape='cylinder')

# Record initial cloth positions for measuring stretch
initial_positions = [vertex for vertex in p.getMeshData(cloth_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)[1]]

# Simulation parameters
selected_robot_index = 0
last_selected_robot_index = selected_robot_index
max_speed = 0.001
acceleration = 0.001
friction = 0.98
control_all = False  # Flag to toggle between individual and all-robot control

# Simulation loop with keyboard control
try:
    while True:
        keys = p.getKeyboardEvents()

        # Check for "a" key to toggle control mode
        if ord("a") in keys and keys[ord("a")] & p.KEY_WAS_TRIGGERED:
            control_all = not control_all
            if not control_all:
                # When switching back to individual control, restore last selected robot
                selected_robot_index = last_selected_robot_index
                print(f"Switched to controlling robot {selected_robot_index + 1}")
            else:
                # Save the current robot index before switching to all-robots control
                last_selected_robot_index = selected_robot_index
                print("Switched to controlling all robots")

        # Control robot(s) based on the current mode
        if control_all:
            # Control all robots simultaneously
            for i in range(N):
                control_robot(robots, helper_objects, visual_spheres, velocities, i, max_speed, acceleration, friction, grasp_height_above_robot)
        else:
            # Control only the selected robot
            selected_robot_index = control_robot(robots, helper_objects, visual_spheres, velocities, selected_robot_index, max_speed, acceleration, friction, grasp_height_above_robot)

        # Measure maximum stretching
        max_stretch = measure_max_stretch(cloth_id, initial_positions)
        print(f"Maximum cloth stretch: {max_stretch}")

        # Check if the cloth is touching the ground
        cloth_touching_ground = is_cloth_touching_ground(cloth_id)
        if cloth_touching_ground:
            print("Cloth is touching the ground.")

        # Check for collisions with obstacles
        for obstacle_id in obstacles:
            contact_points = p.getContactPoints(bodyA=cloth_id, bodyB=obstacle_id)
            if contact_points:
                print(f"Collision detected between cloth and obstacle {obstacle_id}")

        # Step the simulation
        p.stepSimulation()
        time.sleep(1. / 240.)
except KeyboardInterrupt:
    pass

# Disconnect after simulation
p.disconnect()
