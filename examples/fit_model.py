import FlexivPy.robot.interface as interface
import easy_controllers


slow_states = []

robot = interface.Flexiv_client(render=False, create_sim_server=False)
import pickle

with open("array.pkl", "rb") as f:
    states = pickle.load(f)

qs = [s.q for s in states]

take_one_every = 200

qs_reduced = [qs[i] for i in range(0, len(qs), take_one_every)]

print("len qs_reduced", len(qs_reduced))
for i, q in enumerate(qs_reduced):

    controller_status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfiguration(
            qdes=q, max_v=0.5, max_extra_time_rel=1.0
        ),
        dt=0.002,
        max_time=30,
    )

    print("controller_status", controller_status)
    if controller_status != easy_controllers.ControllerStatus.GOAL_REACHED:
        raise ValueError("Controller failed")

    easy_controllers.run_controller(
        robot, easy_controllers.Stay(), dt=0.005, max_time=1.0
    )

    slow_states.append(robot.get_robot_state())

with open("array_slow_v2.pkl", "wb") as f:
    pickle.dump(slow_states, f)
