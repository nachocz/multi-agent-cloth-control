import pybullet as p
import os
from utils import find_closest_cloth_node

def create_cloth(anchor_height):
    """Loads the cloth model and positions it just above the anchor height."""
    cloth_path = os.path.join(os.getcwd(), "square_cloth.obj")
    
    # Set the cloth's initial height slightly above the anchor points
    cloth_position = [0, 0, anchor_height]  # Spawn just above the anchor points
    
    cloth_id = p.loadSoftBody(cloth_path, basePosition=cloth_position, scale=1, mass=1,
                              useNeoHookean=0, useBendingSprings=1, useMassSpring=1,
                              springElasticStiffness=10, springDampingStiffness=0.02,
                              springDampingAllDirections=1, useSelfCollision=1,
                              frictionCoeff=0.5, useFaceContact=1)
    p.changeVisualShape(cloth_id, -1, rgbaColor=[1, 0, 0, 1])
    
    return cloth_id

def anchor_cloth_nodes(cloth_id, helper_objects):
    """Anchors cloth nodes to the helper objects."""
    for helper_id in helper_objects:
        helper_position = p.getBasePositionAndOrientation(helper_id)[0]
        closest_node = find_closest_cloth_node(cloth_id, helper_position)
        p.createSoftBodyAnchor(cloth_id, closest_node, helper_id, -1)
