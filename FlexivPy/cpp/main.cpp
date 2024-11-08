#include <dds/sub/detail/discovery.hpp>
#include <iostream>

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cmath>
#include <flexiv/rdk/gripper.hpp>
#include <iostream>
#include <string>
#include <thread>

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"

#include <flexiv/rdk/model.hpp>
#include <optional>

// TODO: use their approach to terminate the callback!!

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <thread>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/geometry.hpp"

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "hpp/fcl/shape/geometric_shapes.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

#include "pinocchio/collision/collision.hpp"

#include "FlexivData.hpp"
#include "argparse.hpp"
#include "dds/dds.hpp"
#include "utils.hpp"
#include <atomic>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace org::eclipse::cyclonedds;

const std::vector<double> kImpedanceKp = {3000.0, 3000.0, 800.0, 800.0,
                                          200.0,  200.0,  200.0};

const std::vector<double> kImpedanceKd = {80.0, 80.0, 40.0, 40.0,
                                          8.0,  8.0,  8.0};

std::atomic<bool> g_stop_sched = {false};

struct RobotController {

  virtual bool ok() { return robot.operational(); }

  void callback() {

    try {

      auto _cmd = cmd_msg.get();

      if (robot.mode() == flexiv::rdk::Mode::RT_JOINT_POSITION) {

        if (_cmd.mode() != 1) {

          throw_pretty(
              "we are in Joint Position mode, but the command is not of "
              "type Joint Position");
        }
        _send_cmd_position(_cmd);
      } else if (robot_mode == flexiv::rdk::Mode::RT_JOINT_TORQUE) {

        if (_cmd.mode() != 2) {

          throw_pretty("we are in Joint Torque mode, but the command is not of "
                       "type Joint Torque");
        }
        _send_cmd_torque(_cmd);
      } else {
        throw_pretty("Mode not implemented");
      }
      auto _state = _get_state();
      state_msg.update(_state);
    } catch (const std::exception &e) {
      spdlog::error(e.what());
      g_stop_sched = true;
    }
  }

  virtual void start() {
    robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);
    scheduler.AddTask([&] { this->callback(); }, "General Task", 1,
                      scheduler.max_priority());
    scheduler.Start();
  }

  virtual void modify_gripper_cmd(FlexivMsg::FlexivCmd &cmd,
                                  const Eigen::VectorXd &q) {
    cmd.tau_ff().fill(0.);
    cmd.tau_ff_with_gravity() = false;
    cmd.dq().fill(0.);
    std::copy(q.data(), q.data() + q.size(), cmd.q().begin());

    if (robot.mode() == flexiv::rdk::Mode::RT_JOINT_POSITION) {
      cmd.mode() = 1;
    } else if (robot.mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE) {
      cmd.mode() = 2;

      double scaling_kp = 2.;
      double scaling_kv = 2.;

      std::vector<double> t_kImpedanceKp = kImpedanceKp;
      std::vector<double> t_kImpedanceKd = kImpedanceKd;

      scale_vector(t_kImpedanceKp, scaling_kp);
      scale_vector(t_kImpedanceKd, scaling_kv);

      std::copy(t_kImpedanceKp.begin(), t_kImpedanceKp.end(), cmd.kp().begin());
      std::copy(t_kImpedanceKd.begin(), t_kImpedanceKd.end(), cmd.kv().begin());

    } else {

      throw_pretty("Mode not implemented");
    }
  }

  virtual FlexivMsg::FlexivCmd compute_default_cmd(const Eigen::VectorXd &q) {
    FlexivMsg::FlexivCmd cmd;
    cmd.tau_ff().fill(0.);
    cmd.tau_ff_with_gravity() = false;
    cmd.dq().fill(0.);
    std::copy(q.data(), q.data() + q.size(), cmd.q().begin());

    if (robot.mode() == flexiv::rdk::Mode::RT_JOINT_POSITION) {
      cmd.mode() = 1;
    } else if (robot.mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE) {
      cmd.mode() = 2;

      double scaling_kp = 2.;
      double scaling_kv = 2.;

      std::vector<double> t_kImpedanceKp = kImpedanceKp;
      std::vector<double> t_kImpedanceKd = kImpedanceKd;

      scale_vector(t_kImpedanceKp, scaling_kp);
      scale_vector(t_kImpedanceKd, scaling_kv);

      std::copy(t_kImpedanceKp.begin(), t_kImpedanceKp.end(), cmd.kp().begin());
      std::copy(t_kImpedanceKd.begin(), t_kImpedanceKd.end(), cmd.kv().begin());

    } else {

      throw_pretty("Mode not implemented");
    }
    return cmd;
  }

  virtual void stop() {
    std::cout << "stopping scheduler" << std::endl;
    scheduler.Stop();
    std::cout << "stopping robot" << std::endl;
    robot.Stop();
    std::cout << "stopping gripper" << std::endl;
    gripper.Stop();
  }
  RobotController(std::string robot_serial_number,
                  flexiv::rdk::Mode t_robot_mode, bool t_use_gripper,
                  bool check_collisions)
      : robot_serial_number(robot_serial_number), robot(robot_serial_number),
        gripper(robot), use_gripper(t_use_gripper),
        check_collisions(check_collisions) {

    if (t_use_gripper) {
      urdf = "/home/quim/code/alex/tamp_mpc/FlexivPy/FlexivPy/assets/"
             "flexiv_rizon10s_kinematics_w_gripper_mass.urdf";
    } else {
      urdf = "/home/quim/code/alex/tamp_mpc/FlexivPy/FlexivPy/assets/"
             "flexiv_rizon10s_kinematics.urdf";
    }

    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);

    p_ub_endff = Eigen::VectorXd(3);
    p_lb_endff = Eigen::VectorXd(3);
    p_ub_endff << .8, .8, .9;
    p_lb_endff << -.1, -.8, .03;

    if (use_gripper) {
      endeff_name_for_bound_check = "tcp";
    } else {
      endeff_name_for_bound_check = "flange";
    }

    if (std::find_if(model.frames.begin(), model.frames.end(),
                     [&](auto &frame) {
                       return frame.name == endeff_name_for_bound_check;
                     }) == model.frames.end()) {
      throw_pretty("frame not found in model");
    }
    frame_id_for_bound_check = model.getFrameId(endeff_name_for_bound_check);

    pinocchio::urdf::buildGeom(
        model, urdf, pinocchio::COLLISION, geom_model,
        "/home/quim/code/alex/tamp_mpc/FlexivPy/FlexivPy/assets/meshes/");

    geom_model.addAllCollisionPairs();
    if (srdf != "") {
      pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf);
    }

    geom_data = pinocchio::GeometryData(geom_model);

    if (robot.fault()) {
      spdlog::warn(
          "Fault occurred on the connected robot, trying to clear ...");
      // Try to clear the fault
      if (!robot.ClearFault()) {
        spdlog::error("Fault cannot be cleared, exiting ...");
        throw_pretty("Fault cannot be cleared");
      }
      spdlog::info("Fault on the connected robot is cleared");
    }

    spdlog::info("Enabling robot ...");
    robot.Enable();

    while (!robot.operational()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    spdlog::info("Robot is now operational");

    robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

    // TODO: add joint limits that account for the wall and the table!!
    auto bounds = adjust_limits_interval(.02, q_min, q_max);

    q_min = bounds.first;
    q_max = bounds.second;

    tau_min.resize(7);
    tau_max.resize(7);

    tau_max = {100, 100, 50, 50, 30, 30, 30};
    for (size_t i = 0; i < 7; i++) {
      tau_min[i] = -tau_max[i];
    }

    std::cout << "Tau max from robot is " << std::endl;
    print_vector(robot.info().tau_max); /*261 261 123 123 57 57 57 */
    std::cout << "But we limit tau to" << std::endl;
    print_vector(tau_max);

    std::cout << "Real robot created" << std::endl;

    if (use_gripper) {

      spdlog::info(
          "Initializing gripper, this process takes about 10 seconds ...");
      gripper.Init();
      spdlog::info("Initialization complete");
      last_gripper_time = std::chrono::system_clock::now();

      spdlog::info("make sure that the gripper is " + initial_gripper_state);

      if (initial_gripper_state == "open")
        gripper.Move(gripper_width_open, gripper_velocity, gripper_max_force);
      else if (initial_gripper_state == "closed")
        gripper.Move(gripper_width_close, gripper_velocity, gripper_max_force);
      else {
        throw_pretty("Unknown gripper state");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      spdlog::info("Gripper open -- lets go to the main control loop!");
    }

    // Set initial joint positions
    auto init_pos = robot.states().q;
    spdlog::info("Initial joint positions are: {}",
                 flexiv::rdk::utility::Vec2Str(init_pos));

    robot_mode = t_robot_mode;

    state_msg.update(_get_state());
    Eigen::VectorXd q = Eigen::VectorXd::Map(state_msg.get().q().data(), 7);
    cmd_msg.update(compute_default_cmd(q));

    std::cout << "Initial state is" << std::endl;
    std::cout << state_msg.get() << std::endl;

    std::cout << "Initial cmd is" << std::endl;
    std::cout << cmd_msg.get() << std::endl;
  }

  FlexivMsg::FlexivState get_state() { return state_msg.get(); }

  virtual void send_gripper_cmd(const FlexivMsg::FlexivCmd &t_cmd) {

    if (t_cmd.g_cmd() == std::string("open")) {
      if (is_gripper_open()) {
        gripper_opening = false;
        spdlog::info("gripper is already open");
        return;
      } else {
        gripper_opening = true;
        gripper.Move(gripper_width_open, gripper_velocity, gripper_max_force);
        tic_last_gripper_cmd = std::chrono::system_clock::now();
      }
    }

    else if (t_cmd.g_cmd() == std::string("close")) {

      if (is_gripper_closed()) {
        gripper_closing = false;
        spdlog::info("gripper is already closed");
        return;
      }
      if (is_gripper_holding()) {
        gripper_closing = false;
        spdlog::info("gripper is holding");
      } else {
        gripper_closing = true;
        gripper.Move(gripper_width_close, gripper_velocity, gripper_max_force);
      }
    }

    else if (t_cmd.g_cmd() == std::string("stop")) {
      gripper_opening = false;
      gripper_closing = false;
      gripper.Stop();
    }

    else if (t_cmd.g_cmd() == std::string("")) {

    } else {
      throw_pretty("unknown gripper command");
    }
  }

  virtual void send_cmd(const FlexivMsg::FlexivCmd &t_cmd) {

    FlexivMsg::FlexivCmd cmd = t_cmd;

    cmd_msg.update(cmd);
  }

  virtual bool is_gripper_closed() {
    // TODO: force based check!
    return gripper.states().width < gripper_width_close + delta_width;
  }

  bool is_gripper_open() {
    // TODO: force based check!
    return gripper.states().width > gripper_width_open - delta_width;
  }

  bool is_gripper_holding() {
    const bool good_finger_position =
        !is_gripper_closed() && !is_gripper_open();
    const bool gripper_not_moving = !is_gripper_moving();
    const bool good_force = gripper.states().force > gripper_min_closing_force;
    return good_finger_position && gripper_not_moving && good_force;
  }

  bool is_gripper_moving() { return gripper.moving(); }

  virtual FlexivMsg::FlexivCmd get_cmd() { return cmd_msg.get(); }

  virtual void set_robot_state(const std::string &state) {
    robot_state = state;
  }

  virtual std::string get_robot_state() { return robot_state; }

  virtual void parse_config_file(const std::string &filename) {

    // use yaml to open the file.

    std::ifstream file(filename);
    if (!file.good()) {
      throw_pretty("File not found");
    }
    try {
      YAML::Node config = YAML::LoadFile(filename);

      // lets parse the yaml file!
    } catch (const YAML::BadFile &e) {
      std::cerr << "Error loading the YAML file: " << e.what() << std::endl;
    } catch (const YAML::ParserException &e) {
      std::cerr << "YAML parsing error: " << e.what() << std::endl;
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> tic_last_gripper_cmd =
      std::chrono::system_clock::now();

  bool check_endeff_bounds = false;
  Eigen::VectorXd p_ub_endff;
  Eigen::VectorXd p_lb_endff;
  std::string endeff_name_for_bound_check;
  int frame_id_for_bound_check;

  std::string robot_config_file; // define bounds, checks...

  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::GeometryData geom_data;
  pinocchio::GeometryModel geom_model;

  std::string robot_serial_number;

  std::string urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/"
                     "flexiv_rizon10s_kinematics.urdf";
  std::string srdf =
      "/home/quim/code/alex/tamp_mpc/FlexivPy/FlexivPy/assets/r10s_with_capsules.srdf";

  flexiv::rdk::Robot robot;
  flexiv::rdk::Gripper gripper;
  flexiv::rdk::Mode robot_mode = flexiv::rdk::Mode::RT_JOINT_TORQUE;
  flexiv::rdk::Scheduler scheduler;
  bool fake_commands = false;
  bool grav_comp_with_pinocchio = true;
  bool use_gripper = false;

  std::string robot_state = "waiting";
  std::string initial_gripper_state = "closed";

  bool q_gripper_is_moving = false;
  std::chrono::time_point<std::chrono::system_clock> last_gripper_time;
  double gripper_dt_s = .01;
  double gripper_max_force = 20;
  double gripper_velocity = 0.1;
  double gripper_width_open = 0.09;
  double gripper_min_closing_force = 1;
  double gripper_width_close = 0.00;
  double delta_width = .001;

  bool gripper_opening = false;
  bool gripper_closing = false;

  SyncData<FlexivMsg::FlexivCmd> cmd_msg;
  SyncData<FlexivMsg::FlexivState> state_msg;
  Eigen::VectorXd q_stop_for_gripper;

  // First Joint: Use 1.57

  // orginal joint limits are
  /*-2.79257 -2.67037 -2.79257 -2.70527 -2.96707 -1.39627 -2.96707 */
  /*2.79257 2.67037 2.79257 2.70527 2.96707 4.53787 2.96707 */

  std::vector<double> q_min = {-1.57,    -2.4,     -2.79257, -2.70527,
                               -2.96707, -1.39627, -2.96707};

  std::vector<double> q_max = {
      1.57, .75, 2.79257, 2.70527, 2.96707, 4.53787, 2.96707,
  };

  bool overwrite_user_cmd_when_gripper_moving = true;

  std::vector<double> tau_min;
  std::vector<double> tau_max;
  bool check_collisions = false;

  double max_norm_vel = 6;
  double max_norm_acc = 2;
  double max_distance = .3;

  virtual void _send_cmd_torque(const FlexivMsg::FlexivCmd &cmd) {

    // TODO: Preallocate the vectors!
    std::vector<double> torque(7, 0);
    std::vector<double> q = robot.states().q;
    std::vector<double> dq = robot.states().dtheta;

    std::vector<double> target_pose(cmd.q().begin(), cmd.q().end());
    std::vector<double> target_vel(cmd.dq().begin(), cmd.dq().end());

    if (inf_norm(cmd.kp()) > 1e-6) {
      if (euclidean_distance(target_pose, robot.states().q) > max_distance ||
          inf_distance(q, robot.states().q) > max_distance / 2.) {
        std::cout << "ERROR" << std::endl;
        print_vector(target_pose);
        print_vector(robot.states().q);

        throw_pretty("Position is too far from current position");
      }
    }

    if (euclidean_norm(target_vel) > max_norm_vel ||
        inf_norm(target_vel) > max_norm_vel / 2.) {

      std::cout << "target vel is " << std::endl;
      print_vector(target_vel);
      throw_pretty("Velocity norm is too high");
    }

    if (q.size() != 7 || dq.size() != 7 || torque.size() != 7) {

      throw_pretty("Vector size does not match array size!");
    }

    if (cmd.tau_ff().size() != 7 || cmd.kp().size() != 7 ||
        cmd.kv().size() != 7 || cmd.q().size() != 7 || cmd.dq().size() != 7) {

      throw_pretty("Vector size does not match array size!");
    }

    for (size_t i = 0; i < 7; i++) {
      torque[i] = cmd.tau_ff()[i] + cmd.kp()[i] * (cmd.q()[i] - q[i]) +
                  cmd.kv()[i] * (cmd.dq()[i] - dq[i]);
    }

    for (auto &i : torque) {
      if (std::isnan(i) || std::isinf(i)) {
        throw_pretty("Nan in torque");
      }
    }

    for (size_t i = 0; i < torque.size(); i++) {
      if (torque[i] < tau_min[i] or torque[i] > tau_max[i]) {
        print_vector(torque);

        std::cout << "Torque exceded limits at joint" << std::endl;
        std::cout << "i: " << i << " " << torque[i] << " " << tau_min[i] << " "
                  << tau_max[i] << std::endl;
        std::cout << "cmd " << std::endl;
        std::cout << cmd << std::endl;
        std::cout << "state" << std::endl;
        print_vector(q);
        print_vector(dq);
        throw_pretty("Torque is out of limits");
      }
    }

    if (fake_commands) {
      std::cout << "Sending command to robot" << std::endl;
      std::cout << "Torque: ";
      print_vector(torque);
      return;
    } else {

      if (!cmd.tau_ff_with_gravity()) {

        if (grav_comp_with_pinocchio) {
          pinocchio::computeGeneralizedGravity(
              model, data, Eigen::VectorXd::Map(q.data(), q.size()));

          std::vector<double> torque_w_grav = torque;
          for (size_t i = 0; i < 7; i++) {
            torque_w_grav[i] += data.g[i];
          }
          robot.StreamJointTorque(torque_w_grav, false);
        }

        else {
          robot.StreamJointTorque(torque);
        }
      } else {
        robot.StreamJointTorque(torque, false);
      }
    }
  }

  virtual void _send_cmd_position(const FlexivMsg::FlexivCmd &cmd) {

    std::vector<double> target_pos(7, 0);
    std::vector<double> target_vel(7, 0);
    std::vector<double> target_acc(7, 0);

    if (cmd.q().size() == target_pos.size()) {
      std::copy(cmd.q().begin(), cmd.q().end(), target_pos.begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }

    if (cmd.dq().size() == target_vel.size()) {
      std::copy(cmd.dq().begin(), cmd.dq().end(), target_vel.begin());
    } else {

      throw_pretty("Vector size does not match array size!");
    }

    if (q_min.size() != target_pos.size() ||
        q_max.size() != target_pos.size()) {
      throw_pretty("Vector size does not match array size!");
    }

    for (size_t i = 0; i < target_pos.size(); i++) {
      if (target_pos[i] < q_min[i] or target_pos[i] > q_max[i]) {
        print_vector(target_pos);
        throw std::runtime_error("Position is out of limits");
      }
    }

    for (auto &i : target_pos) {
      if (std::isnan(i) or std::isinf(i)) {
        print_vector(target_pos);
        throw std::runtime_error("Nan in target position");
      }
    }

    for (auto &i : target_vel) {
      if (std::isnan(i) or std::isinf(i)) {
        print_vector(target_vel);
        throw_pretty("Nan in target velocity");
      }
    }

    for (auto &i : target_acc) {
      if (std::isnan(i) or std::isinf(i)) {
        print_vector(target_acc);
        throw_pretty("Nan in target acceleration");
      }
    }

    if (euclidean_norm(target_vel) > max_norm_vel) {
      print_vector(target_vel);
      throw_pretty("Velocity norm is too high");
    }

    if (euclidean_norm(target_acc) > max_norm_acc) {
      print_vector(target_acc);
      throw_pretty("Acceleration norm is too high");
    }

    if (euclidean_distance(target_pos, robot.states().q) > max_distance ||
        inf_distance(target_pos, robot.states().q) > max_distance / 2.) {
      print_vector(target_pos);
      print_vector(robot.states().q);

      throw_pretty("Position is too far from current position");
    }

    if (fake_commands) {
      std::cout << "Sending command to robot" << std::endl;
      std::cout << "Position: ";
      print_vector(target_pos);
      std::cout << "Velocity: ";
      print_vector(target_vel);
      std::cout << "Acceleration: ";
      print_vector(target_acc);
      return;
    } else {
      robot.StreamJointPosition(target_pos, target_vel, target_acc);
    }
  }

  virtual FlexivMsg::FlexivState _get_state() {
    FlexivMsg::FlexivState state_out;
    std::string time = get_current_timestamp();
    state_out.timestamp(time);

    auto &q = robot.states().q;
    auto &dq = robot.states().dtheta;
    auto &tau = robot.states().tau;
    auto &ft_sensor = robot.states().ft_sensor_raw;

    Eigen::VectorXd _q = Eigen::VectorXd::Map(q.data(), q.size());

    state_out.state() = robot_state;

    if (q.size() == state_out.q().size()) {
      std::copy(q.begin(), q.end(), state_out.q().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }
    if (dq.size() == state_out.dq().size()) {
      std::copy(dq.begin(), dq.end(), state_out.dq().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }
    if (tau.size() == state_out.tau().size()) {
      std::copy(tau.begin(), tau.end(), state_out.tau().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }

    if (ft_sensor.size() == state_out.ft_sensor().size()) {
      std::copy(ft_sensor.begin(), ft_sensor.end(),
                state_out.ft_sensor().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }

    if (use_gripper) {
      state_out.g_moving() = gripper.moving();
      state_out.g_force() = gripper.states().force;
      state_out.g_width() = gripper.states().width;
      // holding, opening, closing, open, closed
      if (gripper_closing) {
        state_out.g_state() = "closing";
      } else if (gripper_opening) {
        state_out.g_state() = "opening";
      } else if (is_gripper_closed()) {
        state_out.g_state() = "closed";
      } else if (is_gripper_open()) {
        state_out.g_state() = "open";
      } else if (is_gripper_holding()) {
        state_out.g_state() = "holding";
      } else {
        state_out.g_state() = "unknown";
      }
    }

    if (check_collisions) {

      bool is_collision = pinocchio::computeCollisions(model, data, geom_model,
                                                       geom_data, _q, true);

      if (is_collision) {
        throw_pretty("The current state is very close to collision!");
      }
    }
    if (check_endeff_bounds) {

      pinocchio::forwardKinematics(model, data, _q);
      pinocchio::updateFramePlacement(model, data, frame_id_for_bound_check);
      auto M = data.oMf[frame_id_for_bound_check];

      auto p = M.translation();

      if ((p.array() <= p_lb_endff.array()).any() ||
          (p.array() >= p_ub_endff.array()).any()) {
        std::cout << "p is " << p.transpose() << std::endl;
        std::cout << "p_ub_endff is " << p_ub_endff.transpose() << std::endl;
        std::cout << "p_lb_endff is " << p_lb_endff.transpose() << std::endl;
        throw_pretty("the end effector is out of bouns");
      }
    }

    return state_out;
  }
};

// TODO: check if robot is operating!!!
int main(int argc, char *argv[]) {

  // Initialize variables with default values
  int control_mode = 2;
  std::string serial_number = "Rizon10s-062147";
  bool use_gripper = false;
  bool ignore_collisions = true; // TODO!! -- URDF with Collision Meshes!!
  double dt = .001;
  double stop_dt_if_no_msg_ms = 50;
  bool profile = false;
  double max_time_s = std::numeric_limits<double>::infinity();
  std::string robot_config_file = "";
  bool read_cmd_if_moving_gripper = false;


  // Create an ArgumentParser object with program description
  argparse::ArgumentParser program(
      "Robot C++ Bridge",
      "Bridge Between Python Controller and C++ Real-time Torque Controller");

  // Define options
  program.add_argument("-h", "--help")
      .help("Print help")
      .default_value(false)
      .implicit_value(true)
      .nargs(0);

  program.add_argument("-cm", "--control_mode")
      .help("0: Error, 1: Kin, 2: Torque")
      .default_value(control_mode)
      .store_into(control_mode); // Store directly into control_mode

  program.add_argument("-s", "--serial_number")
      .help("Serial Number of Robot")
      .default_value(serial_number)
      .store_into(serial_number); // Store directly into serial_number

  program.add_argument("-g", "--gripper")
      .help("Use Gripper")
      .default_value(false)
      .implicit_value(true)
      .store_into(use_gripper); // Store directly into use_gripper

  program.add_argument("-i", "--ignore_collisions")
      .help("Ignore Collisions")
      .default_value(false)
      .implicit_value(true)
      .store_into(ignore_collisions);

  program.add_argument("--dt")
      .help("Time step")
      .default_value(dt)
      .store_into(dt);

  program.add_argument("-stop_dt", "--stop_dt_if_no_msg_ms")
      .help("Stop dt if no message received")
      .default_value(stop_dt_if_no_msg_ms)
      .store_into(stop_dt_if_no_msg_ms);

  program.add_argument("-p", "--profile")
      .help("Profile time")
      .default_value(false)
      .implicit_value(true)
      .store_into(profile);

  program.add_argument("-mt", "--max_time")
      .help("Max time in seconds")
      .default_value(max_time_s)
      .store_into(max_time_s);

  program.add_argument("-rcf", "--robot_config_file")
      .help("Robot config file")
      .default_value(robot_config_file)
      .store_into(robot_config_file);

  program.add_argument("-rcmg", "--read_cmd_if_moving_gripper")
      .help("Read user command if moving gripper")
      .default_value(read_cmd_if_moving_gripper)
      .implicit_value(true)
      .store_into(read_cmd_if_moving_gripper);


  try {
    // Parse the command-line arguments
    program.parse_args(argc, argv);

    // Print help if requested
    if (program.get<bool>("--help")) {
      std::cout << program.help().str() << std::endl;
      exit(0);
    }

  } catch (const std::runtime_error &err) {
    std::cout << "Error parsing the command line arguments" << std::endl;
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  if (robot_config_file != "") {
    throw_pretty("Robot config file not implemented");
  }

  std::cout << "=== [Subscriber] Create reader." << std::endl;

  /* First, a domain participant is needed.
   * Create one on the default domain. */
  dds::domain::DomainParticipant participant(domain::default_id());

  /* To subscribe to something, a topic is needed. */
  dds::topic::Topic<FlexivMsg::FlexivCmd> topic_cmd(participant, "FlexivCmd");

  /* A reader also needs a subscriber. */
  dds::sub::Subscriber subscriber(participant);

  /* Now, the reader can be created to subscribe to a HelloWorld message.
   */
  dds::sub::DataReader<FlexivMsg::FlexivCmd> reader(subscriber, topic_cmd);

  /* To publish something, a topic is needed. */
  dds::topic::Topic<FlexivMsg::FlexivState> topic_state(participant,
                                                        "FlexivState");

  /* A writer also needs a publisher. */
  dds::pub::Publisher publisher(participant);

  /* Now, the writer can be created to publish a HelloWorld message. */
  dds::pub::DataWriter<FlexivMsg::FlexivState> writer(publisher, topic_state);

  std::cout << "=== [Subscriber] Wait for message." << std::endl;

  FlexivMsg::FlexivCmd cmd_msg;
  FlexivMsg::FlexivState state_msg;
  std::vector<double> last_good_state;
  Eigen::VectorXd waiting_q(7), gripper_waiting_q(7);

  flexiv::rdk::Mode t_robot_mode;

  if (control_mode == 0) {
    throw_pretty("Error mode not implemented");
  } else if (control_mode == 1) {
    t_robot_mode = flexiv::rdk::Mode::RT_JOINT_POSITION;
  } else if (control_mode == 2) {
    t_robot_mode = flexiv::rdk::Mode::RT_JOINT_TORQUE;
  }

  auto robot = std::make_shared<RobotController>(
      serial_number, t_robot_mode, use_gripper, !ignore_collisions);

  robot->start();
  state_msg = robot->get_state();
  waiting_q = Eigen::VectorXd::Map(state_msg.q().data(), state_msg.q().size());
  writer.write(state_msg);

  auto tic = std::chrono::high_resolution_clock::now();
  auto last_msg_time = std::chrono::high_resolution_clock::now();

  std::vector<double> time_loop_us_vec;

  robot->set_robot_state("waiting");
  spdlog::info("Mode: Waiting");

  auto elapsed_s = [&] {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - tic)
               .count() /
           1000.;
  };

  bool receiving_gripper_cmds = false;
  while (elapsed_s() < max_time_s && !g_stop_sched && robot->ok()) {

    auto tic_loop = std::chrono::high_resolution_clock::now();

    std::optional<FlexivMsg::FlexivCmd> maybe_cmd = read_last_cmd(reader);

    if (maybe_cmd) {
      last_msg_time = tic_loop;
      cmd_msg = *maybe_cmd;
    }

    if (robot->get_robot_state() == "user" && !maybe_cmd &&
        std::chrono::duration_cast<std::chrono::milliseconds>(tic_loop -
                                                              last_msg_time)
                .count() > stop_dt_if_no_msg_ms) {
      // change mode!!
      spdlog::info("No message received for a while -- entering waiting mode");
      waiting_q =
          Eigen::VectorXd::Map(state_msg.q().data(), state_msg.q().size());
      robot->set_robot_state("waiting");
    }

    if (robot->get_robot_state() == "waiting" && maybe_cmd) {
      spdlog::info("Received command -- entering user mode");
      robot->set_robot_state("user");
    }

    if (robot->get_robot_state() == "user" && maybe_cmd) {
      // pass
    }

    if (use_gripper) {
      if (cmd_msg.g_cmd().size()) {
        if (!receiving_gripper_cmds) {
          spdlog::info("receiving gripper commands");
          receiving_gripper_cmds = true;
          gripper_waiting_q =
              Eigen::VectorXd::Map(state_msg.q().data(), state_msg.q().size());
          spdlog::info("gripper_waiting_q is");
          std::cout << gripper_waiting_q.transpose() << std::endl;
        }
        robot->send_gripper_cmd(cmd_msg);

        if (!read_cmd_if_moving_gripper) {
          robot->modify_gripper_cmd(cmd_msg, gripper_waiting_q);
        }
      } else {
        receiving_gripper_cmds = false;
      }
    }

    if (robot->get_robot_state() == "waiting") {
      if (control_mode == 2) {
        cmd_msg = robot->compute_default_cmd(waiting_q);
      } else if (control_mode == 1) {
        cmd_msg = robot->compute_default_cmd(waiting_q);
      }
    }

    robot->send_cmd(cmd_msg);
    state_msg = robot->get_state();
    // lets add semantic information in state msg!
    state_msg.state(robot->get_robot_state());
    writer.write(state_msg);

    int time_loop_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::high_resolution_clock::now() - tic_loop)
                           .count();

    if (time_loop_us > 1e6 * dt) {
      spdlog::warn("Our Loop took too long: {} us", time_loop_us);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(
        std::max(0, static_cast<int>(1e6 * dt - time_loop_us))));

    if (profile) {
      time_loop_us_vec.push_back(time_loop_us);
    }
  }

  robot->stop();

  if (profile) {
    std::cout << "Time loop us: " << std::endl;
    std::cout << "Max: "
              << *std::max_element(time_loop_us_vec.begin(),
                                   time_loop_us_vec.end())
              << std::endl;
    std::cout << "Min: "
              << *std::min_element(time_loop_us_vec.begin(),
                                   time_loop_us_vec.end())
              << std::endl;
    std::cout << "Average: "
              << std::accumulate(time_loop_us_vec.begin(),
                                 time_loop_us_vec.end(), 0) /
                     time_loop_us_vec.size()
              << std::endl;
  }

  return EXIT_SUCCESS;
}
