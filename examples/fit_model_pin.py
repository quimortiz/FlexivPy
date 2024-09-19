import numpy as np

import pickle


from pinocchio.robot_wrapper import RobotWrapper
import numpy as np


with open("array_slow_v2.pkl", "rb") as f:
    data = pickle.load(f)


urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/modified_robot.urdf"
meshes_dir = "/home/quim/code/FlexivPy/FlexivPy/assets/meshes/"

robot = RobotWrapper.BuildFromURDF(urdf, meshes_dir)


d = data[0]

tau_g = robot.gravity(np.array(d.q))
x0 = np.array([i.mass for i in robot.model.inertias])

reg_weight = 0.001


def error(x):
    for i in range(8):
        robot.model.inertias[i].mass = x[i]
    error = 0
    for d in data:
        tau_g = robot.gravity(np.array(d.q))
        tau_real = d.tau
        dif = tau_g - tau_real
        error += np.dot(dif, dif)
    error /= len(data)
    error += reg_weight * np.dot(x - x0, x - x0)

    return error


from scipy.optimize import minimize

print(error(x0))
min_res = minimize(error, x0=x0, method="BFGS")

print(min_res)
print(x0)


def error_with_cm(x):
    for i in range(8):
        robot.model.inertias[i].mass = x[i]
    # then I have them [x,y,z;x,y,z,...]

    start_index = 8
    for i in range(8):
        p = x[start_index + i * 3 : start_index + (i + 1) * 3]
        robot.model.inertias[i].lever = p

    error = 0
    for d in data:
        tau_g = robot.gravity(np.array(d.q))
        tau_real = d.tau
        dif = tau_g - tau_real
        error += np.dot(dif, dif)
    error /= len(data)
    error += reg_weight * np.dot(x - x0, x - x0)

    return error


from scipy.optimize import minimize

x0 = np.zeros(8 + 8 * 3)
for i in range(8):
    x0[i] = robot.model.inertias[i].mass

start_index = 8
for i in range(8):
    x0[start_index + i * 3 : start_index + (i + 1) * 3] = robot.model.inertias[i].lever

print(error_with_cm(x0))
min_res = minimize(error_with_cm, x0=x0, method="BFGS")

print(min_res)
print(x0)

error(min_res.x)
print("difference is", min_res.x - x0)

print("evaluating at optimum")

robot.model.saveToXML("/tmp/robot.xml", "flexiv")


import xml.etree.ElementTree as ET

# Load the URDF file
tree = ET.parse(urdf)
root = tree.getroot()


D = {
    "base_link": 0,
    "link1": 1,
    "link2": 2,
    "link3": 3,
    "link4": 4,
    "link5": 5,
    "link6": 6,
    "link7": 7,
}


xsol = min_res.x

# Loop through each link and find the inertial elements
for link in root.findall("link"):
    inertial = link.find("inertial")

    if inertial is not None:
        # Modify the mass
        mass_element = inertial.find("mass")
        if mass_element is not None:
            mass_element.set("value", str(xsol[D[link.get("name")]]))
        # Modify the inertia's origin (xyz)
        origin_element = inertial.find("origin")
        if origin_element is not None:
            start_index = 8
            p = xsol[
                start_index
                + D[link.get("name")] * 3 : start_index
                + (D[link.get("name")] + 1) * 3
            ]
            print("p is", p)
            pstr = " ".join([str(i) for i in p])
            origin_element.set("xyz", pstr)  # Set new xyz value

tree.write("modified_robot.urdf")


# # robot.model.inertias[2].mass
# robot.model.inertias[2].lever
#
# for i,ii in enumerate(robot.model.inertias):
#     print('index ', i)
#     print(ii.lever)
#     print(ii.mass)
#
#
#
#
# print('pinocchio', tau_g)
# print('experimental', d.tau)
# print('difference')
#
# for i in range(8):
#     robot.model.inertias[i].mass += 1.
#
# tau_g2 = robot.gravity(np.array(d.q))
# print(tau_g2)

# import pdb; pdb.set_trace()
