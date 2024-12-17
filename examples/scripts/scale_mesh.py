import struct
import pathlib


def scale_obj_file(input_file, output_file, scale_factors):
    with open(input_file, "r") as f:
        lines = f.readlines()

    with open(output_file, "w") as f:
        for line in lines:
            if line.startswith("v "):
                parts = line.split()
                # Apply scale factors for X, Y, and Z respectively
                scaled_parts = [
                    str(float(parts[1]) * scale_factors[0]),  # Scale X
                    str(float(parts[2]) * scale_factors[1]),  # Scale Y
                    str(float(parts[3]) * scale_factors[2]),  # Scale Z
                ]
                f.write(f"v {' '.join(scaled_parts)}\n")
            else:
                f.write(line)


def scale_stl_file(input_file, output_file, scale_factors):
    # scale_factors is a tuple or list of three floats: [sx, sy, sz]
    with open(input_file, "rb") as f_in:
        # Read the header (80 bytes)
        header = f_in.read(80)

        # Read number of triangles (4 bytes, unsigned int)
        num_triangles_bytes = f_in.read(4)
        num_triangles = struct.unpack("<I", num_triangles_bytes)[0]

        # Prepare to read each triangle
        # Each triangle: 12 bytes normal + 36 bytes vertices + 2 bytes attribute
        # Normal: 3 floats (3*4 bytes)
        # Vertices: 3 vertices * (3 floats each) = 9 floats (36 bytes)
        # Attribute: 2 bytes

        triangles = []
        for _ in range(num_triangles):
            # Read normal (3 floats)
            normal_data = f_in.read(12)
            normal = struct.unpack("<3f", normal_data)

            # Read vertices (9 floats)
            vertex_data = f_in.read(36)
            vertices = list(struct.unpack("<9f", vertex_data))

            # Scale vertices
            # vertices is [x1, y1, z1, x2, y2, z2, x3, y3, z3]
            for i in range(0, 9, 3):
                vertices[i] *= scale_factors[0]  # x
                vertices[i + 1] *= scale_factors[1]  # y
                vertices[i + 2] *= scale_factors[2]  # z

            # Read attribute
            attribute_data = f_in.read(2)

            # Store the modified triangle
            triangles.append((normal, vertices, attribute_data))
        # Write the scaled STL
    with open(output_file, "wb") as f_out:
        # Write header
        f_out.write(header)
        # Write number of triangles
        f_out.write(struct.pack("<I", num_triangles))

        # Write each triangle
        for normal, vertices, attribute_data in triangles:
            f_out.write(struct.pack("<3f", *normal))
            f_out.write(struct.pack("<9f", *vertices))
            f_out.write(attribute_data)


files = [
    "spring_link.stl",
    "silicone_pad.stl",
    "pad.stl",
    "follower.stl",
    "coupler.stl",
    "driver.stl",
    "spring_link.stl",
    "silicone_pad.stl",
    "base_mount.stl",
    "base.stl",
]

base = "/home/FlexivPy/FlexivPy/assets/meshes/"

pathlib.Path(f"{base}/gripper_s").mkdir(exist_ok=True, parents=True)


for f in files:
    scale_stl_file(f"{base}/gripper/{f}", f"{base}/gripper_s/{f}", [0.001] * 3)
