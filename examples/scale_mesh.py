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


input_file = (
    "/home/quim/flexiv_rdk_new/flexiv_rdk/resources/meshes/rizon10s/visual/ring.obj"
)
output_file = (
    "/home/quim/flexiv_rdk_new/flexiv_rdk/resources/meshes/rizon10s/visual/ring_s.obj"
)

scale_factors = (0.079, 0.079, 0.002)


D = {
    "1": [0.079, 0.079, 0.002],
    "2": [0.079, 0.079, 0.002],
    "3": [0.062, 0.062, 0.002],
    "4": [0.062, 0.062, 0.002],
    "5": [0.051, 0.051, 0.002],
    "6": [0.051, 0.051, 0.002],
    "7": [0.051, 0.051, 0.002],
}


for key, value in D.items():
    output_file = f"/home/quim/flexiv_rdk_new/flexiv_rdk/resources/meshes/rizon10s/visual/ring_s{key}.obj"
    scale_obj_file(input_file, output_file, value)
