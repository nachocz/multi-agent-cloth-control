import pybullet as p
import math  # Import math for trigonometric functions

def initialize_environment():
    """Sets up gravity and resets the simulation."""
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setGravity(0, 0, -9.8)

def add_ground_plane():
    """Adds a ground plane."""
    return p.loadURDF("plane.urdf")

def add_obstacles(shape='cylinder'):
    """Adds obstacles (prisms or cylinders) to the scene at fixed positions."""
    obstacle_ids = []
    
    # Define fixed positions and parameters for each obstacle
    fixed_positions = [
        (1.5, 1.5, 0.5),
        (-1.5, 1.5, 0.5),
        (1.5, -1.5, 0.5),
        (-1.5, -1.5, 0.5),
        (0, 2.5, 0.5)
    ]
    
    for pos in fixed_positions:
        x, y, z = pos

        if shape == 'cylinder':
            # Create a cylinder obstacle
            radius, height = 0.2, 0.7
            collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
            visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[0.3, 0.3, 0.8, 1])
        elif shape == 'prism':
            # Create a prism (box) obstacle
            half_extents = [0.2, 0.2, 0.5]
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.8, 0.3, 0.3, 1])
        else:
            print(f"Unknown shape: {shape}. Using 'cylinder' as default.")
            collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.2, height=0.5)
            visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.2, length=0.5, rgbaColor=[0.3, 0.3, 0.8, 1])

        # Create the obstacle in the simulation
        obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape,
                                        baseVisualShapeIndex=visual_shape, basePosition=[x, y, z])
        obstacle_ids.append(obstacle_id)
    
    return obstacle_ids
