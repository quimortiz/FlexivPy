import os

ASSETS_PATH = os.path.join(os.path.dirname(__file__), "assets")
URDF_PATH = os.path.join(os.path.dirname(__file__), "assets/mjmodel.xml")
XML_PATH = os.path.join(
    os.path.dirname(__file__), "assets/flexiv_rizon10s_kinematics_w_gripper_mass.urdf"
)
MESHES_PATH = os.path.join(os.path.dirname(__file__), "assets/meshes")


def getDataPath():
    resdir = os.path.join(os.path.dirname(__file__))
    return resdir
