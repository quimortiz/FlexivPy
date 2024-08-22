#include <iostream>

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
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

/*
 * Copyright(c) 2006 to 2021 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
 */
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

#include <atomic>

/* Include the C++ DDS API. */
#include "dds/dds.hpp"

/* Include data type and specific traits to be used with the C++ DDS API. */
#include "FlexivData.hpp"
/*#include "HelloWorldData.hpp"*/

#include <fstream>

using namespace org::eclipse::cyclonedds;

const std::vector<double> kImpedanceKp = {3000.0, 3000.0, 800.0, 800.0,
                                          200.0,  200.0,  200.0};

const std::vector<double> kImpedanceKd = {80.0, 80.0, 40.0, 40.0,
                                          8.0,  8.0,  8.0};

std::atomic<bool> g_stop_sched = {false};

// Helper function to format the error message with file, line, and function
// details
std::string format_error_message(const std::string &message, const char *file,
                                 int line, const char *func) {
  std::ostringstream oss;
  oss << "Error: " << message << "\n"
      << "File: " << file << "\n"
      << "Line: " << line << "\n"
      << "Function: " << func << "\n";
  return oss.str();
}
std::pair<std::vector<double>, std::vector<double>>
adjust_limits_interval(double margin, const std::vector<double> &q_min,
                       const std::vector<double> &q_max) {
  // Check that the vectors are of the same size
  if (q_min.size() != q_max.size()) {
    throw std::invalid_argument("q_min and q_max must have the same size");
  }

  std::vector<double> new_q_min(q_min.size());
  std::vector<double> new_q_max(q_max.size());

  for (size_t i = 0; i < q_min.size(); ++i) {
    double margin_i =
        margin * (q_max[i] - q_min[i]); // 2.5% of the range on each side
    new_q_min[i] = q_min[i] + margin_i;
    new_q_max[i] = q_max[i] - margin_i;
  }

  return std::make_pair(new_q_min, new_q_max);
}

void scale_vector(std::vector<double> &vec, double scale) {
  for (auto &v : vec) {
    v *= scale;
  }
}

bool is_edge_collision(pinocchio::Model &model, pinocchio::Data &data,
                       pinocchio::GeometryModel &geom_model,
                       pinocchio::GeometryData &geom_data,
                       const Eigen::VectorXd &q, const Eigen::VectorXd &qgoal,
                       int num_checks = 100) {
  bool is_collision = false;
  for (size_t i = 0; i < num_checks; i++) {
    Eigen::VectorXd q_i = q + (qgoal - q) * i / num_checks;

    is_collision = pinocchio::computeCollisions(model, data, geom_model,
                                                geom_data, q_i, true);

    if (is_collision) {
      std::cout << "Edge collision!" << std::endl;
      break;
    }
  }
  return is_collision;
}

std::vector<Eigen::VectorXd>
go_to_q(const Eigen::VectorXd &q, const Eigen::VectorXd &qgoal,
        const Eigen::VectorXd &qmin, const Eigen::VectorXd &qmax,
        pinocchio::Model &model, pinocchio::Data &data,
        pinocchio::GeometryModel &geom_model,
        pinocchio::GeometryData &geom_data, int num_checks = 100) {

  std::vector<Eigen::VectorXd> path;

  const int num_collision_checks = 100;
  bool is_collision = is_edge_collision(model, data, geom_model, geom_data, q,
                                        qgoal, num_collision_checks);

  if (is_collision) {

    const int num_attempts = 50;
    Eigen::VectorXd best_q_rand(7);
    double best_cost = std::numeric_limits<double>::infinity();

    std::cout << "lets try to go first to a random point, then to home "
              << std::endl;

    Eigen::VectorXd q_rand(7);
    bool path_found = false;
    for (size_t j = 0; j < num_attempts; j++) {
      q_rand = qmin + (qmax - qmin)
                              .cwiseProduct(Eigen::VectorXd::Random(7) +
                                            Eigen::VectorXd::Ones(7)) *
                          .5;

      if (!is_edge_collision(model, data, geom_model, geom_data, q, q_rand,
                             num_collision_checks) &&
          !is_edge_collision(model, data, geom_model, geom_data, q_rand, qgoal,
                             num_collision_checks)) {
        path_found = true;
        double tentative_cost =
            (qgoal - q_rand).norm() + (q_rand - qgoal).norm();
        if (tentative_cost < best_cost) {
          best_cost = tentative_cost;
          best_q_rand = q_rand;
        }
      }
    }

    if (!path_found) {
      spdlog::error("Cannot find a path to home -- move robot manually");
    } else {
      path = {best_q_rand, qgoal};
    }

  } else {
    std::cout << "going home directly!" << std::endl;
    path = {qgoal};
  }
  return path;
}
// Macro to throw an exception with detailed information
#define throw_pretty(message)                                                  \
  throw std::runtime_error(                                                    \
      format_error_message(message, __FILE__, __LINE__, __PRETTY_FUNCTION__))

std::string get_current_timestamp() {
  // Get the current time
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  // Extract the milliseconds part
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;

  // Convert time_t to a tm structure
  std::tm tm_now = *std::localtime(&time_t_now);

  // Create a stringstream to format the time
  std::stringstream timestamp;
  timestamp << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S") << '.'
            << std::setfill('0') << std::setw(3) << milliseconds.count();

  return timestamp.str();
}

#include <mutex>

template <typename T> class SyncData {
public:
  // Constructor
  SyncData() = default;

  // Constructor with initial value
  explicit SyncData(const T &initial_value) : data(initial_value) {}

  // Update the data with mutex protection
  void update(const T &new_value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    data = new_value;
  }

  // Retrieve the data with mutex protection
  T get() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return data;
  }

private:
  mutable std::mutex data_mutex; // Mutex to protect the data
  T data;                        // The data being protected
};

struct Robot {

  virtual FlexivMsg::FlexivState get_state() = 0;

  virtual void send_cmd(const FlexivMsg::FlexivCmd &cmd) = 0;

  virtual void compute_default_cmd(const Eigen::VectorXd &q,
                                   FlexivMsg::FlexivCmd &cmd) {
    (void)q;
    (void)cmd;
  }

  virtual void compute_home_cmd(const FlexivMsg::FlexivState &state,
                                FlexivMsg::FlexivCmd &cmd) {
    (void)cmd;
  }

  virtual bool ok() { return true; }
  virtual void start() {}
  virtual void stop() {}
  virtual FlexivMsg::FlexivCmd get_cmd() { return {}; }
  virtual ~Robot() {}
  bool home_mode = false;

  Eigen::VectorXd qhome;

  virtual bool is_home() { return false; }
  virtual void reset_home() {}
  virtual void set_robot_state(const std::string &state) {
    (void)state;
    throw_pretty("Not implemented");
  }

  virtual std::string get_robot_state() { throw_pretty("Not implemented"); }
};

struct Fake_robot : public Robot {

public:
  std::ofstream log_file_state;
  std::ofstream log_file_cmd;
  bool write_to_file;
  Fake_robot(bool write_file = true) : write_to_file(write_file) {
    std::cout << "Fake robot created" << std::endl;

    if (write_to_file) {
      std::cout << "Fake robot is writing to file" << std::endl;

      log_file_state.open("log_file_state.txt");
      log_file_state << "Fake robot created at " << get_current_timestamp()
                     << std::endl;

      log_file_cmd.open("log_file_cmd.txt");
      log_file_cmd << "Fake robot created at" << get_current_timestamp()
                   << std::endl;
    }
  }
  virtual FlexivMsg::FlexivState get_state() override {
    FlexivMsg::FlexivState state;
    std::string time = get_current_timestamp();
    state.timestamp(time);

    if (write_to_file) {
      log_file_state << "**" << std::endl;
      log_file_state << "Getting state from robot at " << time << std::endl;
      log_file_state << "Message has time stamp: " << time << std::endl;
    }
    return state;
  }
  virtual void send_cmd(const FlexivMsg::FlexivCmd &cmd) override {
    std::string time = get_current_timestamp();
    if (write_to_file) {
      log_file_cmd << "**" << std::endl;
      log_file_cmd << "Sending command to robot at " << time << std::endl;
      std::string msg_time = cmd.timestamp();
      log_file_cmd << "Message has time stamp: " << msg_time << std::endl;
    }
  }

  ~Fake_robot() {
    if (write_to_file) {
      log_file_state << "Fake robot destroyed at" << get_current_timestamp()
                     << std::endl;
      log_file_state.close();
      log_file_cmd << "Fake robot destroyed at" << get_current_timestamp()
                   << std::endl;
    }
  }
};

double euclidean_norm(const std::vector<double> &vec) {
  double norm = 0;
  for (auto &v : vec) {
    norm += v * v;
  }
  return std::sqrt(norm);
}

double inf_norm(const std::vector<double> &vec) {
  double norm = 0;
  for (auto &v : vec) {
    norm = std::max(norm, std::abs(v));
  }
  return norm;
}

double euclidean_distance(const std::vector<double> &vec1,
                          const std::vector<double> &vec2) {
  double dist = 0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
  }
  return std::sqrt(dist);
}
// TODO: fix this mess!
double euclidean_distance(const std::array<double, 7> &vec1,
                          const std::array<double, 7> &vec2) {
  double dist = 0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
  }
  return std::sqrt(dist);
}

template <typename T> double inf_norm(const T &vec) {
  double norm = 0;
  for (auto &v : vec) {
    norm = std::max(norm, std::abs(v));
  }
  return norm;
}

double inf_distance(const std::vector<double> &vec1,
                    const std::vector<double> &vec2) {
  double dist = 0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist = std::max(dist, std::abs(vec1[i] - vec2[i]));
  }
  return dist;
}

void print_vector(const std::vector<double> &vec) {
  for (auto &v : vec) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
}

struct RealRobot : Robot {

  virtual bool ok() override { return robot.operational(); }
  flexiv::rdk::Robot robot;
  flexiv::rdk::Gripper gripper;
  flexiv::rdk::Mode robot_mode = flexiv::rdk::Mode::RT_JOINT_POSITION;
  std::unique_ptr<flexiv::rdk::Scheduler> scheduler;
  bool fake_commands = false;
  bool grav_comp_with_pinocchio = true; // TODO: with grav comp in pinocchio,
  // i have observed some issues if the scheduler fails!

  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::GeometryData geom_data;
  pinocchio::GeometryModel geom_model;
  std::vector<Eigen::VectorXd> recovery_path;
  int go_home_subgoal = 0;
  double delta_home = .005;

  virtual bool is_home() override {
    auto _q = robot.states().q;
    Eigen::VectorXd q = Eigen::VectorXd::Map(_q.data(), 7);
    return (q - qhome).norm() < delta_home;
  }

  virtual void reset_home() override {
    recovery_path.clear();
    go_home_subgoal = 0;
  }

  virtual void compute_home_cmd(const FlexivMsg::FlexivState &state,
                                FlexivMsg::FlexivCmd &cmd) override {

    // Check if I have a recovery path

    if (!recovery_path.size()) {
      spdlog::info("No recovery path available, computing one!!");

      Eigen::VectorXd q = Eigen::VectorXd::Map(state.q().data(), 7);
      Eigen::VectorXd qgoal = qhome;
      Eigen::VectorXd qmin = Eigen::VectorXd::Map(q_min.data(), 7);
      Eigen::VectorXd qmax = Eigen::VectorXd::Map(q_max.data(), 7);

      recovery_path =
          go_to_q(q, qgoal, qmin, qmax, model, data, geom_model, geom_data);
      go_home_subgoal = 0;
    }

    spdlog::info("I have a recovery path, executing it");

    Eigen::VectorXd qgoal = recovery_path.at(go_home_subgoal);
    Eigen::VectorXd q = Eigen::VectorXd::Map(robot.states().q.data(), 7);

    std::cout << "qgoal - q" << std::endl;
    std::cout << (qgoal - q).norm() << std::endl;
    if ((qgoal - q).norm() < delta_home) {
      spdlog::info("Reached subgoal " + std::to_string(go_home_subgoal));
      go_home_subgoal++;
      go_home_subgoal =
          std::max(go_home_subgoal, int(recovery_path.size()) - 1);
      qgoal = recovery_path.at(go_home_subgoal);
    }

    // get the current state
    double dx_limit = .01;

    if ((qgoal - q).norm() > dx_limit) {
      qgoal = q + (qgoal - q).normalized() * dx_limit;
    }

    std::copy(qgoal.data(), qgoal.data() + qgoal.size(), cmd.q().begin());
    std::fill(cmd.dq().begin(), cmd.dq().end(), 0.);
    std::fill(cmd.tau_ff().begin(), cmd.tau_ff().end(), 0.);
    cmd.tau_ff_with_gravity() = false;

    double scaling_kp = 1.;
    double scaling_kv = 1.5;

    std::vector<double> t_kImpedanceKp = kImpedanceKp;
    std::vector<double> t_kImpedanceKd = kImpedanceKd;

    scale_vector(t_kImpedanceKp, scaling_kp);
    scale_vector(t_kImpedanceKd, scaling_kv);

    std::copy(t_kImpedanceKp.begin(), t_kImpedanceKp.end(), cmd.kp().begin());
    std::copy(t_kImpedanceKd.begin(), t_kImpedanceKd.end(), cmd.kv().begin());
    cmd.mode() = 2;
  }

  // it true, we just print the commands to the screen.

  // this is called by the scheduler
  //
  //
  //
  void callback() {

    try {

      auto _cmd = cmd_msg.get();

      if (robot_mode == flexiv::rdk::Mode::RT_JOINT_POSITION) {

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
      /*_get_state(_state);*/
      state_msg.update(_state);
    } catch (const std::exception &e) {
      spdlog::error(e.what());
      g_stop_sched = true;
    }
  }

  virtual void start() override {
    scheduler = std::make_unique<flexiv::rdk::Scheduler>();
    switch_mode_sync(flexiv::rdk::Mode::RT_JOINT_TORQUE);
    /*scheduler = flexiv::rdk::Scheduler();*/
    scheduler->AddTask([&] { this->callback(); }, "General Task", 1,
                       scheduler->max_priority());
    scheduler->Start();
  }

  virtual void compute_default_cmd(const Eigen::VectorXd &q,
                                   FlexivMsg::FlexivCmd &cmd) override {

    cmd.tau_ff().fill(0.);
    cmd.tau_ff_with_gravity() = false;
    cmd.dq().fill(0.);
    std::copy(q.data(), q.data() + q.size(), cmd.q().begin());

    if (robot_mode == flexiv::rdk::Mode::RT_JOINT_POSITION) {
      cmd.mode() = 1;
    } else if (robot_mode == flexiv::rdk::Mode::RT_JOINT_TORQUE) {
      cmd.mode() = 2;

      double scaling_kp = 1.;
      double scaling_kv = 1.;

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

  virtual void stop() override {
    std::cout << "stopping scheduler" << std::endl;
    scheduler->Stop();
    std::cout << "stopping robot" << std::endl;
    robot.Stop();
    std::cout << "stopping gripper" << std::endl;
    gripper.Stop();
  }

  RealRobot(std::string robot_serial_number, flexiv::rdk::Mode t_robot_mode)
      : robot(robot_serial_number), gripper(robot) {

    qhome.resize(7);
    qhome << 0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000;

    std::string urdf =
        "/home/quim/code/FlexivPy/FlexivPy/assets/flexiv_rizon10s_kinematics.urdf";

    std::string srdf =
        "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.srdf";

    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    pinocchio::urdf::buildGeom(model, urdf, pinocchio::COLLISION, geom_model,
                                "/home/quim/code/FlexivPy/FlexivPy/assets/meshes/" );

    geom_model.addAllCollisionPairs();
    if (srdf != "") {
      pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf);
    }

    geom_data = pinocchio::GeometryData(geom_model);

    // Clear fault on the connected robot if any
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

    // Enable the robot, make sure the E-stop is released before enabling
    spdlog::info("Enabling robot ...");
    robot.Enable();

    // Wait for the robot to become operational
    while (!robot.operational()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    spdlog::info("Robot is now operational");

    // check if i can compute gravity compensation term.

    if (false) {
      auto _q = robot.states().q;
      auto q = Eigen::VectorXd::Map(_q.data(), _q.size());
      std::cout << "q: " << q.transpose() << std::endl;
      pinocchio::computeGeneralizedGravity(model, data, q);
      std::cout << "Pinocchio grav. compensation:" << data.g.transpose()
                << std::endl;

      flexiv::rdk::Model f_model(robot);

      std::vector<double> qv(q.data(), q.data() + q.size());
      std::vector<double> vel(7, 0.);

      f_model.Update(qv, vel);

      // Compute gravity vector
      auto g = f_model.g();

      std::cout << "g from them " << g.transpose() << std::endl;

      std::cout << "Diference" << std::endl;
      std::cout << g - data.g << std::endl;

      Eigen::VectorXd relative_difference =
          (g - data.g).cwiseAbs().cwiseQuotient(g.cwiseAbs());
      std::cout << relative_difference << std::endl;
    }

    auto bounds =
        adjust_limits_interval(.02, robot.info().q_min, robot.info().q_max);

    std::cout << "tau max from robot is " << std::endl;
    print_vector(robot.info().tau_max);

    q_min = bounds.first;
    q_max = bounds.second;

    tau_min.resize(7);
    tau_max.resize(7);

    // Tau max from robot is:
    /*261 261 123 123 57 57 57 */
    tau_max = {100, 100, 50, 50, 30, 30, 30};
    for (size_t i = 0; i < 7; i++) {
      tau_min[i] = -tau_max[i];
    }
    std::cout << "we limit tau to" << std::endl;
    print_vector(tau_max);
    print_vector(tau_min);

    std::cout << "Real robot created" << std::endl;

    switch_mode_sync(flexiv::rdk::Mode::NRT_JOINT_POSITION);

    // Instantiate gripper control interface

    // Manually initialize the gripper, not all grippers need this step
    
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

    // Set initial joint positions
    auto init_pos = robot.states().q;
    spdlog::info("Initial joint positions set to: {}",
                 flexiv::rdk::utility::Vec2Str(init_pos));

    robot_mode = t_robot_mode;

    FlexivMsg::FlexivState tmp_state = _get_state();
    FlexivMsg::FlexivCmd tmp_cmd;

    Eigen::VectorXd q = Eigen::VectorXd::Map(tmp_state.q().data(), 7);
    compute_default_cmd(q, tmp_cmd);

    std::cout << "Initial cmd is" << std::endl;
    std::cout << tmp_cmd << std::endl;

    std::cout << "Initial state is" << std::endl;
    std::cout << tmp_state << std::endl;

    state_msg.update(tmp_state);
    cmd_msg.update(tmp_cmd);
  }

  void switch_mode_sync(flexiv::rdk::Mode t_robot_mode) {
    robot.SwitchMode(t_robot_mode);
    robot_mode = t_robot_mode;
  }

  FlexivMsg::FlexivState get_state() override { return state_msg.get(); }

  virtual void send_cmd(const FlexivMsg::FlexivCmd &t_cmd) override {

    // rewrite this so that i have a small state machine!!
    // Openning, Open, Closing, Close,
    // Openning and Closing are moving.
    //
    //

    // update the moving status

    FlexivMsg::FlexivCmd cmd = t_cmd;
    if (gripper_available) {
      if (gripper_openning && is_gripper_open()) {
        spdlog::info("gripper was openning and now is open");
        gripper_openning = false;
        // TODO: should I call stop?
      }

      if (gripper_closing && is_gripper_closed()) {
        spdlog::info("gripper was closing and now is closed");
        gripper_closing = false;
        // TODO: should I call stop?
      }

      gripper_moving = gripper_openning || gripper_closing;

      if (is_gripper_closed() && t_cmd.g_cmd() == std::string("close")) {
        // do nothing
      }

      else if (!gripper_moving && !is_gripper_open() &&
               t_cmd.g_cmd() == std::string("open")) {
        gripper.Move(gripper_width_open, gripper_velocity, gripper_max_force);
        std::cout << "Opening gripper..." << std::endl;
        gripper_openning = true;
        spdlog::info("updating _state_stop_for_gripper");
        q_stop_for_gripper =
            Eigen::VectorXd::Map(state_msg.get().q().data(), 7);
      }

      else if (!is_gripper_closed() && t_cmd.g_cmd() == std::string("close")) {
        gripper.Move(gripper_width_close, gripper_velocity, gripper_max_force);
        gripper_closing = true;
        spdlog::info("start closing motion: updating _state_stop_for_gripper");
        q_stop_for_gripper =
            Eigen::VectorXd::Map(state_msg.get().q().data(), 7);
      }

      if (gripper_openning || gripper_closing) {
        if (overwrite_user_cmd_when_gripper_moving) {
          std::cout << "gripper is moving, overriting cmd" << std::endl;
          compute_default_cmd(q_stop_for_gripper, _cmd);
          FlexivMsg::FlexivState tmp_state = state_msg.get();
          Eigen::VectorXd q = Eigen::VectorXd::Map(tmp_state.q().data(), 7);
          if ((q_stop_for_gripper - q).norm() > .2) {
            throw_pretty("The gripper is moving, but the stop state is too far "
                         "from the current state");
          }
        }
      }
    }

    /*spdlog::info("Sending command to robot");*/
    /*std::cout << cmd << std::endl;*/
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

  bool is_gripper_moving() { return gripper.moving(); }

  virtual FlexivMsg::FlexivCmd get_cmd() override { return cmd_msg.get(); }

  virtual void set_robot_state(const std::string &state) override {
    robot_state = state;
  }

  virtual std::string get_robot_state() override { return robot_state; }

private:
  std::string robot_state = "waiting";
  std::string initial_gripper_state = "closed";

  bool q_gripper_is_moving = false;
  std::chrono::time_point<std::chrono::system_clock> last_gripper_time;
  double gripper_dt_s = .01;
  double gripper_max_force = 20;
  double gripper_velocity = 0.1;
  double gripper_width_open = 0.09;
  double gripper_width_close = 0.02;
  double delta_width = .001;

  bool gripper_moving = false;
  bool gripper_openning = false;
  bool gripper_closing = false;

  SyncData<FlexivMsg::FlexivCmd> cmd_msg;
  SyncData<FlexivMsg::FlexivState> state_msg;
  Eigen::VectorXd q_stop_for_gripper;

  std::vector<double> q_min;
  std::vector<double> q_max;
  bool overwrite_user_cmd_when_gripper_moving = true;

  std::vector<double> tau_min;
  std::vector<double> tau_max;
  bool check_collisions = false;
  bool gripper_available = true;

  /*bool gripper_is_moving = false;*/

  FlexivMsg::FlexivCmd
      _cmd; // just to avoid memory allocation. Do not use outside callback!
  FlexivMsg::FlexivState
      _state; // just to avoid memory allocation. Do not use outside callback!

  double max_norm_vel = 6;
  double max_norm_acc = 2;
  double max_distance = .3;

  virtual void _send_cmd_torque(const FlexivMsg::FlexivCmd &cmd) {

    /*std::cout << "Sending torque command" << std::endl;*/
    /*std::cout << cmd << std::endl;*/

    // TODO: Preallocate the vectors!
    std::vector<double> torque(7, 0);
    std::vector<double> q = robot.states().q;
    std::vector<double> dq = robot.states().dtheta;

    std::vector<double> target_pose(cmd.q().begin(), cmd.q().end());
    std::vector<double> target_vel(cmd.dq().begin(), cmd.dq().end());

    // check that q is close the current position

    if (inf_norm(cmd.kp()) > 1e-6) {
      if (euclidean_distance(target_pose, robot.states().q) > max_distance ||
          inf_distance(q, robot.states().q) > max_distance / 2.) {
        std::cout << "ERROR" << std::endl;
        print_vector(target_pose);
        print_vector(robot.states().q);

        /*robot.Stop();*/
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

      /*robot.Stop();*/
      throw_pretty("Vector size does not match array size!");
    }

    if (cmd.tau_ff().size() != 7 || cmd.kp().size() != 7 ||
        cmd.kv().size() != 7 || cmd.q().size() != 7 || cmd.dq().size() != 7) {

      /*robot.Stop();*/
      throw_pretty("Vector size does not match array size!");
    }

    for (size_t i = 0; i < 7; i++) {
      torque[i] = cmd.tau_ff()[i] + cmd.kp()[i] * (cmd.q()[i] - q[i]) +
                  cmd.kv()[i] * (cmd.dq()[i] - dq[i]);
    }

    // check that the desired velocity is close to 0.

    // check that there are non NAns or infs

    for (auto &i : torque) {
      if (std::isnan(i) || std::isinf(i)) {

        /*robot.Stop();*/
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

        /*robot.Stop();*/
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
          // compute gravity compenstation with pinocchio
          pinocchio::computeGeneralizedGravity(
              model, data, Eigen::VectorXd::Map(q.data(), q.size()));

          std::vector<double> torque_w_grav = torque;
          for (size_t i = 0; i < 7; i++) {
            torque_w_grav[i] += data.g[i];
          }

          /*std::cout << "final torque would be " << std::endl;*/
          /*print_vector(final_torque);*/
          /*robot.StreamJointTorque(torque, false);*/

          /*spdlog::info("sending torque");*/
          /*print_vector(torque);*/
          robot.StreamJointTorque(torque_w_grav, false);
        }

        else {
          /*spdlog::info("sending torque");*/
          /*print_vector(torque);*/
          robot.StreamJointTorque(torque);
        }
      } else {
        robot.StreamJointTorque(torque, false);
      }

      // check that the torque is inside limits
    }
  }

  virtual void _send_cmd_position(const FlexivMsg::FlexivCmd &cmd) {

    std::vector<double> target_pos(7, 0);
    std::vector<double> target_vel(7, 0);
    std::vector<double> target_acc(7, 0);

    if (cmd.q().size() == target_pos.size()) {
      std::copy(cmd.q().begin(), cmd.q().end(), target_pos.begin());
    } else {

      /*robot.Stop();*/
      throw_pretty("Vector size does not match array size!");
    }

    if (cmd.dq().size() == target_vel.size()) {
      std::copy(cmd.dq().begin(), cmd.dq().end(), target_vel.begin());
    } else {

      /*robot.Stop();*/
      throw_pretty("Vector size does not match array size!");
    }

    // Lots of sanity checks!!

    if (q_min.size() != target_pos.size() ||
        q_max.size() != target_pos.size()) {

      /*robot.Stop();*/
      throw_pretty("Vector size does not match array size!");
    }

    for (size_t i = 0; i < target_pos.size(); i++) {
      if (target_pos[i] < q_min[i] or target_pos[i] > q_max[i]) {
        throw std::runtime_error("Position is out of limits");
      }
    }

    for (auto &i : target_pos) {
      if (std::isnan(i) or std::isinf(i)) {
        throw std::runtime_error("Nan in target position");
      }
    }

    for (auto &i : target_vel) {
      if (std::isnan(i) or std::isinf(i)) {

        /*robot.Stop();*/
        throw_pretty("Nan in target velocity");
      }
    }

    for (auto &i : target_acc) {
      if (std::isnan(i) or std::isinf(i)) {

        /*robot.Stop();*/
        throw_pretty("Nan in target acceleration");
      }
    }

    if (euclidean_norm(target_vel) > max_norm_vel) {

      /*robot.Stop();*/
      throw_pretty("Velocity norm is too high");
    }

    if (euclidean_norm(target_acc) > max_norm_acc) {

      /*robot.Stop();*/
      throw_pretty("Acceleration norm is too high");
    }

    if (euclidean_distance(target_pos, robot.states().q) > max_distance ||
        inf_distance(target_pos, robot.states().q) > max_distance / 2.) {
      print_vector(target_pos);
      print_vector(robot.states().q);

      /*robot.Stop();*/
      throw_pretty("Position is too far from current position");
    }

    // TODO: check the reading of the velocity!
    /*if (euclidean_distance(target_vel, robot.states().dtheta) >
     * max_distance_vel or*/
    /*    inf_distance(target_vel, robot.states().dtheta) > max_distance_vel
     * / 2.)
     * {*/
    /*  {*/
    /*    throw_pretty("Velocity is too far from current Velocity");*/
    /*  }*/
    /**/
    /*}*/

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

    // add the special state of the robot.
    //

    state_out.state() = robot_state;

    if (q.size() == state_out.q().size()) {
      std::copy(q.begin(), q.end(), state_out.q().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }
    if (dq.size() == state_out.dq().size()) {
      std::copy(dq.begin(), dq.end(), state_out.dq().begin());
    } else {

      /*robot.Stop();*/
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

    // add info about the gripper

    if (gripper_available) {
      state_out.g_moving() = gripper.moving();
      state_out.g_force() = gripper.states().force;
      state_out.g_width() = gripper.states().width;
      if (gripper_closing) {
        state_out.g_state() = "closing";
      } else if (gripper_openning) {
        state_out.g_state() = "openning";
      } else if (is_gripper_closed()) {
        state_out.g_state() = "closed";
      } else if (is_gripper_open()) {
        state_out.g_state() = "open";
      } else {
        state_out.g_state() = "unknown";
      }
    }

    if (check_collisions) {
      Eigen::VectorXd _q = Eigen::VectorXd::Map(q.data(), q.size());

      bool is_collision = pinocchio::computeCollisions(model, data, geom_model,
                                                       geom_data, _q, true);

      if (is_collision) {
        /*robot.Stop();*/
        throw_pretty("The current state is very close to collision!");
      }
    }
    return state_out;
  }
};

std::optional<FlexivMsg::FlexivCmd>
read_last_cmd(dds::sub::DataReader<FlexivMsg::FlexivCmd> &reader) {
  FlexivMsg::FlexivCmd cmd;
  bool good_msg = false;

  dds::sub::LoanedSamples<FlexivMsg::FlexivCmd> samples;

  /* Try taking samples from the reader. */
  samples = reader.take();

  if (samples.length() > 0) {
    /* Use an iterator to run over the set of samples. */
    dds::sub::LoanedSamples<FlexivMsg::FlexivCmd>::const_iterator sample_iter;
    for (sample_iter = samples.begin(); sample_iter < samples.end();
         ++sample_iter) {
      /* Get the message and sample information. */
      const FlexivMsg::FlexivCmd &msg = sample_iter->data();
      const dds::sub::SampleInfo &info = sample_iter->info();

      /* Sometimes a sample is read, only to indicate a data
       * state change (which can be found in the info). If
       * that's the case, only the key value of the sample
       * is set. The other data parts are not.
       * Check if this sample has valid data. */
      if (info.valid()) {
        good_msg = true;
        cmd = msg;
      }
    }
  }

  if (good_msg) {
    return cmd;
  } else {
    return {};
  }
}

// TODO: check if robot is operating!!!
int main(int argc, char *argv[]) {

  if (argc != 4) {
    std::cerr
        << "Usage: " << argv[0]
        << " <{0:fake,1:real}> <robot_serial_number> <0:Error,1:Kin,2:Torque>"
        << std::endl;
    return 1; // Return a non-zero value to indicate an error
  }

  // Extract the robot serial number from the command-line arguments
  int real_mode = std::stoi(argv[1]);
  std::string robot_serial_number = argv[2];
  int operation_mode = std::stoi(argv[3]);

  try {

    std::cout << "=== [Subscriber] Create reader." << std::endl;

    const double dt = .001;
    const bool go_home = false;
    double stop_dt_if_no_msg_ms = 50;
    double max_time_s = std::numeric_limits<double>::infinity() ; 
    double dt_us = dt * 1e6;

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
    Eigen::VectorXd waiting_q(7);

    std::shared_ptr<Robot> robot;

    if (real_mode == 0) {
      bool write_to_file = false;
      robot = std::make_shared<Fake_robot>(write_to_file);
    } else if (real_mode == 1) {

      flexiv::rdk::Mode t_robot_mode;

      if (operation_mode == 0) {
        throw_pretty("Error mode not implemented");
      } else if (operation_mode == 1) {
        t_robot_mode = flexiv::rdk::Mode::RT_JOINT_POSITION;
      } else if (operation_mode == 2) {
        t_robot_mode = flexiv::rdk::Mode::RT_JOINT_TORQUE;
      }

      else {
        throw_pretty("Operation mode not implemented");
      }
      robot = std::make_shared<RealRobot>(robot_serial_number, t_robot_mode);
    }

    robot->start();
    state_msg = robot->get_state();
    waiting_q =
        Eigen::VectorXd::Map(state_msg.q().data(), state_msg.q().size());
    std::cout << "First state msg is \n" << state_msg << std::endl;
    writer.write(state_msg);

    auto tic = std::chrono::high_resolution_clock::now();
    auto last_msg_time = std::chrono::high_resolution_clock::now();
    std::vector<double> time_loop_us_vec;

    if (go_home) {
      robot->set_robot_state("going-home");
      spdlog::info("Mode: Going home");
    } else {
      robot->set_robot_state("waiting");
      spdlog::info("Mode: Waiting");
    }

    auto elapsed_s = [&] {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::high_resolution_clock::now() - tic)
                 .count() /
             1000.;
    };

    while (elapsed_s() < max_time_s && !g_stop_sched && robot->ok()) {

      auto tic_loop = std::chrono::high_resolution_clock::now();

      std::optional<FlexivMsg::FlexivCmd> maybe_cmd = read_last_cmd(reader);

      if (maybe_cmd) {
        last_msg_time = tic_loop;
        cmd_msg = *maybe_cmd;
        std::cout << "Received command" << std::endl;
        std::cout << cmd_msg << std::endl;
      }

      if (robot->get_robot_state() == "user" && !maybe_cmd &&
          std::chrono::duration_cast<std::chrono::milliseconds>(tic_loop -
                                                                last_msg_time)
                  .count() > stop_dt_if_no_msg_ms) {
        // change mode!!
        spdlog::info(
            "No message received for a while -- entering waiting mode");
        waiting_q =
            Eigen::VectorXd::Map(state_msg.q().data(), state_msg.q().size());
        robot->set_robot_state("waiting");
      }

      if ((robot->get_robot_state() == "waiting" ||
           robot->get_robot_state() == "waiting-home") &&
          maybe_cmd) {
        spdlog::info("Received command -- entering user mode");
        robot->set_robot_state("user");
      }

      if (robot->get_robot_state() == "user" && maybe_cmd) {
        std::cout << "checing ... " << std::endl;
        if (cmd_msg.special_cmd() == "go-home") {
          spdlog::info("Received go-home command");
          robot->set_robot_state("going-home");
        }
      }

      if (robot->get_robot_state() == "going-home") {
        // Check if I am already home!

        if (robot->is_home()) {
          spdlog::info("Already home");
          spdlog::info("Entering waiting mode");
          robot->set_robot_state("waiting-home");
        } else {
          std::cout << "computing go gome cmd" << std::endl;
          robot->compute_home_cmd(state_msg, cmd_msg);
        }
      }

      if (robot->get_robot_state() == "waiting") {
        if (operation_mode == 2) {
          robot->compute_default_cmd(waiting_q, cmd_msg);
        } else if (operation_mode == 1) {
          throw_pretty("Joint position mode not implemented");
          /*robot->compute_default_cmd(state_msg, cmd_msg);*/
        }
      }

      if (robot->get_robot_state() == "waiting-home") {
        if (operation_mode == 2) {
          robot->compute_default_cmd(robot->qhome, cmd_msg);
        } else if (operation_mode == 1) {
          throw_pretty("Joint position mode not implemented");
          /*robot->compute_default_cmd(state_msg, cmd_msg);*/
        }
      }

      /*std::cout << "final cmd is" << std::endl;*/
      /*std::cout << cmd_msg << std::endl;*/
      robot->send_cmd(cmd_msg);
      state_msg = robot->get_state();
      // lets add semantic information in state msg!
      state_msg.state(robot->get_robot_state());
      writer.write(state_msg);

      int time_loop_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now() - tic_loop)
              .count();

      time_loop_us_vec.push_back(time_loop_us);
      if (time_loop_us > 1e6 * dt) {
        std::cout << "Our Loop took too long: " << time_loop_us << " us"
                  << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(
          std::max(0, static_cast<int>(dt_us - time_loop_us))));
    }

    robot->stop();

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
  } catch (const dds::core::Exception &e) {
    std::cerr << "=== [Subscriber] DDS exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception &e) {
    std::cerr << "=== [Subscriber] C++ exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "=== Done." << std::endl;

  return EXIT_SUCCESS;
}
