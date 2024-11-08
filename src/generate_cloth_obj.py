def generate_square_cloth_obj(filename, num_nodes_per_side):
    """
    Generates a square cloth mesh in .obj format with a red color.
    
    Args:
        filename (str): Name of the .obj file to save (without extension).
        num_nodes_per_side (int): Number of nodes per side of the square.
                                  Must be >= 2 for a valid cloth mesh.
    """
    if num_nodes_per_side < 2:
        raise ValueError("num_nodes_per_side must be at least 2.")
    
    # Calculate the spacing between nodes
    spacing = 1.0 / (num_nodes_per_side - 1)
    
    # Generate vertices
    vertices = []
    for i in range(num_nodes_per_side):
        for j in range(num_nodes_per_side):
            x = j * spacing - 0.5  # Center the cloth at origin (shift by -0.5)
            y = i * spacing - 0.5  # Center the cloth at origin (shift by -0.5)
            z = 0
            vertices.append((x, y, z))
    
    # Generate faces (each face is two triangles)
    faces = []
    for i in range(num_nodes_per_side - 1):
        for j in range(num_nodes_per_side - 1):
            # Indices of the four corners of the square face
            v0 = i * num_nodes_per_side + j
            v1 = v0 + 1
            v2 = v0 + num_nodes_per_side
            v3 = v2 + 1
            # Create two triangles for the quad face
            faces.append((v0 + 1, v1 + 1, v3 + 1))  # .obj indices are 1-based
            faces.append((v0 + 1, v3 + 1, v2 + 1))

    # Write the .obj file with material reference
    obj_filename = f"{filename}.obj"
    with open(obj_filename, 'w') as obj_file:
        
        # Write vertices
        for v in vertices:
            obj_file.write(f"v {v[0]} {v[1]} {v[2]}\n")
        
        # Write faces
        for face in faces:
            obj_file.write(f"f {face[0]} {face[1]} {face[2]}\n")
    
    print(f"{obj_filename} generated with {num_nodes_per_side} nodes per side and red color.")

# Example usage
filename = "square_cloth"
num_nodes_per_side = 10  # Adjust the number of nodes as needed

generate_square_cloth_obj(filename, num_nodes_per_side)
