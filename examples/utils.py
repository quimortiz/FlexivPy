
import pinocchio as pin
import numpy as np

def frame_error(q, robot, frame_id, oMdes):
    data = robot.data
    robot.framePlacement(q, frame_id, update_kinematics=True)
    iMd = data.oMf[frame_id].actInv(oMdes)
    err = pin.log(iMd).vector  # in joint frame
    return np.dot(err, err)


def frame_error_jac(q, robot, frame_id, oMdes):
    model = robot.model
    data = robot.data
    robot.framePlacement(q, frame_id, update_kinematics=True)
    iMd = data.oMf[frame_id].actInv(oMdes)
    err = pin.log(iMd).vector  # in joint frame
    J = pin.computeFrameJacobian(model, data, q, frame_id)  # in joint frame
    Jlog = pin.Jlog6(iMd.inverse())
    J = -Jlog @ J
    return 2 * J.T @ err


def fit_3rd_order_polynomial(x, x_dot, y, y_dot):
    """
    p(0) = x
    p'(0) = x_dot
    p(1) = y
    p'(1) = y_dot
    """
    A = np.array(
        [
            [1, 0, 0, 0],  # p(0) = x
            [0, 1, 0, 0],  # p'(0) = x_dot
            [1, 1, 1, 1],  # p(1) = y
            [0, 1, 2, 3],
        ]
    )  # p'(1) = y_dot

    # Initialize the array to store the polynomial coefficients
    coefficients = np.zeros((4, 7))

    # For each dimension in R^7, solve the linear system for the coefficients
    for i in range(7):
        B = np.array([x[i], x_dot[i], y[i], y_dot[i]])
        coefficients[:, i] = np.linalg.solve(A, B)

    return coefficients


def evaluate_polynomial(coefficients, t):
    """
    return f, f_dot, f_ddot
    """

    assert t >= 0 and t <= 1
    f = np.zeros(7)
    df = np.zeros(7)
    dff = np.zeros(7)
    for i in range(7):
        a_0, a_1, a_2, a_3 = coefficients[:, i]
        f[i] = a_3 * t**3 + a_2 * t**2 + a_1 * t + a_0
        df[i] = 3 * a_3 * t**2 + 2 * a_2 * t + a_1
        dff[i] = 6 * a_3 * t + 2 * a_2
    return f, df, dff


def polynomial_max_velocity(coefficients):
    # component wise

    max_velocity = np.zeros(7)
    time_of_max_velocity = np.zeros(7)

    for i in range(7):
        a_0, a_1, a_2, a_3 = coefficients[:, i]

        # Compute velocity at t = 0 and t = 1
        velocity_at_t0 = a_1
        velocity_at_t1 = 3 * a_3 + 2 * a_2 + a_1

        candidates_v = np.array([abs(velocity_at_t0), abs(velocity_at_t1)])
        candidates_t = np.array([0.0, 1.0])

        # Find the critical point where the second derivative (acceleration) is zero
        if abs(a_3) > 1e-6:  # Only if a_3 is non-zero do we have a critical point
            t_critical = -a_2 / (3 * a_3)
        else:
            t_critical = None
        # Evaluate velocity at the critical point, if it lies in the interval [0, 1]
        if t_critical is not None and 0 <= t_critical <= 1:
            velocity_at_critical = (
                3 * a_3 * t_critical**2 + 2 * a_2 * t_critical + a_1
            )
            candidates_v = np.append(candidates_v, abs(velocity_at_critical))
            candidates_t = np.append(candidates_t, t_critical)

        max_index = np.argmax(candidates_v)
        max_velocity[i] = candidates_v[max_index]
        time_of_max_velocity[i] = candidates_t[max_index]

    return max_velocity, time_of_max_velocity
