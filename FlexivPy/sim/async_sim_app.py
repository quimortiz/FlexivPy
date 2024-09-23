from FlexivPy.sim.async_engine import AsyncSimManager
from FlexivPy.sim.MuJoCo import FlexivSimMujoco
from FlexivPy.robot.model.pinocchio import FlexivModel
import numpy as np
import argparse
import time
import signal

running = True
def signal_handler(sig, frame):
    running = False
    
def main():
    argp = argparse.ArgumentParser()
    argp.add_argument("--render", action="store_true", default=True, help="render the simulation")
    argp.add_argument(
        "--config", type=str, default="FlexivPy/config/robot.yaml", help="config file"
    )
    argp.add_argument("--render_images", action="store_true", help="render images")
    argp.add_argument("--xml_path", type=str, default=None, help="xml path")
    argp.add_argument("--timeout", type=float, default=np.inf, help="simulation duration")
    argp.add_argument("--urdf", type=str, default=None, help="urdf path")
    argp.add_argument(
        "--meshes_dir", type=str, default=None, help="meshes directrory path"
    )
    argp.add_argument(
        "--joints", type=str, nargs="+", default=None, help="Names of the joints"
    )
    argp.add_argument(
        "--has_gripper", action="store_true", help="render the simulation"
    )
    argp.add_argument("--camera_name", type=str, default="static_camera")
    args = argp.parse_args()

    q0 = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
    dt = 0.001
    robot_model = FlexivModel(
        render=False,
        q0=q0,
        urdf=args.urdf,
        meshes_dir=args.meshes_dir,
    )

    simulator = FlexivSimMujoco(
                                dt=dt,
                                render=args.render,
                                xml_path=args.xml_path,
                                q0=q0,
                                pin_model=robot_model.robot,
                                render_images=args.render_images,
                                joint_names=args.joints,
                                has_gripper=args.has_gripper,
                                camera_name=args.camera_name
                                    )
    sim = AsyncSimManager(simulator, timeout=args.timeout)
    signal.signal(signal.SIGINT, signal_handler)
    while sim.running:
        if running:
            time.sleep(0.5)
        else:
            sim.terminate()
            time.sleep(1)

if __name__ == "__main__":
    main()