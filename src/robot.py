import pybullet as p
import numpy as np

def create_robots(N, robot_height, grasp_height_above_robot, radius):
    """Creates robots, helper objects, and visual spheres."""
    corner_angles = [np.pi / 4, 3 * np.pi / 4, 5 * np.pi / 4, 7 * np.pi / 4]

    robots = []
    helper_objects = []
    visual_spheres = []
    velocities = [[0, 0] for _ in range(N)]

    for i in range(N):
        angle = corner_angles[i]
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)

        # Create the robot
        robot_id = create_robot([x, y, robot_height])
        robots.append(robot_id)

        # Create helper and visual objects at the grasp height above the robot
        helper_position = [x, y, robot_height + grasp_height_above_robot]
        helper_id, visual_sphere_id = create_helper_objects(helper_position)
        helper_objects.append(helper_id)
        visual_spheres.append(visual_sphere_id)

    return robots, helper_objects, visual_spheres, velocities

def create_robot(position):
    """Creates a single robot at the given position with prismatic constraints."""
    robot_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05])
    robot_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05], rgbaColor=[0, 0, 1, 1])
    robot_id = p.createMultiBody(baseMass=1000, baseCollisionShapeIndex=robot_shape,
                                 baseVisualShapeIndex=robot_visual, basePosition=position)

    # Limit robot movement to x and y axes by creating constraints, then setting max force
    constraint_x = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_PRISMATIC, [1, 0, 0], [0, 0, 0], [0, 0, 0])
    p.changeConstraint(constraint_x, maxForce=5000)
    
    constraint_y = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_PRISMATIC, [0, 1, 0], [0, 0, 0], [0, 0, 0])
    p.changeConstraint(constraint_y, maxForce=5000)
    
    return robot_id

def create_helper_objects(position):
    """Creates helper and visual spheres for each robot."""
    helper_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.01)
    helper_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1])
    helper_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=helper_shape,
                                  baseVisualShapeIndex=helper_visual, basePosition=position)
    
    visual_sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.5])
    visual_sphere_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_sphere_visual, basePosition=position)
    
    return helper_id, visual_sphere_id

def control_robot(robots, helper_objects, visual_spheres, velocities, selected_robot_index, max_speed, acceleration, friction, grasp_height_above_robot):
    """Controls robot movements based on keyboard inputs."""
    keys = p.getKeyboardEvents()

    # Check if number keys (1-9) are pressed to select a robot
    for i in range(len(robots)):
        if ord(str(i + 1)) in keys and keys[ord(str(i + 1))] & p.KEY_IS_DOWN:
            selected_robot_index = i
            print(f"Controlling robot {i + 1}")

    # Update and apply velocities for selected robot
    for i, robot_id in enumerate(robots):
        x_velocity, y_velocity = velocities[i]
        if i == selected_robot_index:
            x_velocity, y_velocity = update_velocity(keys, x_velocity, y_velocity, acceleration, max_speed)
        velocities[i] = [x_velocity * friction, y_velocity * friction]
        
        # Move robots and corresponding objects
        new_position = p.getBasePositionAndOrientation(robot_id)[0]
        new_position = [new_position[0] + velocities[i][0], new_position[1] + velocities[i][1], new_position[2]]
        
        p.resetBasePositionAndOrientation(robot_id, new_position, p.getBasePositionAndOrientation(robot_id)[1])
        p.resetBasePositionAndOrientation(helper_objects[i], [new_position[0], new_position[1], new_position[2] + grasp_height_above_robot], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(visual_spheres[i], [new_position[0], new_position[1], new_position[2] + grasp_height_above_robot], [0, 0, 0, 1])

    return selected_robot_index  # Return the updated index

def update_velocity(keys, x_velocity, y_velocity, acceleration, max_speed):
    """Updates velocity based on arrow key inputs."""
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        y_velocity = min(y_velocity + acceleration, max_speed)
    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        y_velocity = max(y_velocity - acceleration, -max_speed)
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        x_velocity = max(x_velocity - acceleration, -max_speed)
    elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        x_velocity = min(x_velocity + acceleration, max_speed)
    return x_velocity, y_velocity
