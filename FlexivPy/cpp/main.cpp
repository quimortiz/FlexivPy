#include <iostream>

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

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

  // Retrieve the data with mutex protection and output it to the provided
  // reference
  void get(T &output) const {
    std::lock_guard<std::mutex> lock(data_mutex);
    output = data;
  }

private:
  mutable std::mutex data_mutex; // Mutex to protect the data
  T data;                        // The data being protected
};

struct Robot {
  virtual void get_state(FlexivMsg::FlexivState &state) = 0;
  virtual void send_cmd(const FlexivMsg::FlexivCmd &cmd) = 0;

  virtual void compute_default_cmd(const FlexivMsg::FlexivState &state,
                                   FlexivMsg::FlexivCmd &cmd) {
    (void)cmd;
  }

  virtual void start() {}
  virtual void stop() {}
  virtual void get_cmd(FlexivMsg::FlexivCmd &cmd) { (void)cmd; }

  virtual ~Robot() {}
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
  virtual void get_state(FlexivMsg::FlexivState &state) override {
    std::string time = get_current_timestamp();
    state.timestamp(time);

    if (write_to_file) {
      log_file_state << "**" << std::endl;
      log_file_state << "Getting state from robot at " << time << std::endl;
      log_file_state << "Message has time stamp: " << time << std::endl;
    }
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
  flexiv::rdk::Robot robot;
  flexiv::rdk::Mode robot_mode = flexiv::rdk::Mode::RT_JOINT_POSITION;
  flexiv::rdk::Scheduler scheduler;
  bool fake_commands = false;
  // it true, we just print the commands to the screen.

  // this is called by the scheduler
  //
  //
  //
  void callback() {
    cmd_msg.get(_cmd);
    if (robot_mode == flexiv::rdk::Mode::RT_JOINT_POSITION) {

      // continue here!!
      // use mode to indicate the type of control!
      if (_cmd.mode() != 1) {
        throw_pretty("we are in Joint Position mode, but the command is not of "
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
    _get_state(_state);
    state_msg.update(_state);
  }

  virtual void start() override {
    scheduler.AddTask([&] { this->callback(); }, "General Task", 1,
                      scheduler.max_priority());
    scheduler.Start();
  }

  virtual void compute_default_cmd(const FlexivMsg::FlexivState &state,
                                   FlexivMsg::FlexivCmd &cmd) override {

    cmd.tau_ff().fill(0.);
    cmd.dq().fill(0.);
    cmd.q() = state.q();

    if (robot_mode == flexiv::rdk::Mode::RT_JOINT_POSITION) {
      cmd.mode() = 1;
    } else if (robot_mode == flexiv::rdk::Mode::RT_JOINT_TORQUE) {
      cmd.mode() = 2;

      double scaling_kp = .4;
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

  virtual void stop() override { scheduler.Stop(); }

  RealRobot(std::string robot_serial_number, flexiv::rdk::Mode t_robot_mode,
            bool go_home)
      : robot(robot_serial_number) {
    std::cout << "Real robot created" << std::endl;

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
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::info("Robot is now operational");

    // Move robot to home pose
    if (go_home) {
      spdlog::info("Moving to home pose");
      switch_mode_sync(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
      robot.ExecutePrimitive("Home()");

      // Wait for the primitive to finish
      while (robot.busy()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    switch_mode_sync(t_robot_mode);

    // Set initial joint positions
    auto init_pos = robot.states().q;
    spdlog::info("Initial joint positions set to: {}",
                 flexiv::rdk::utility::Vec2Str(init_pos));

    FlexivMsg::FlexivState tmp_state;
    FlexivMsg::FlexivCmd tmp_cmd;

    _get_state(tmp_state);
    compute_default_cmd(tmp_state, tmp_cmd);

    std::cout << "Initial cmd is" << std::endl;
    std::cout << tmp_cmd << std::endl;

    state_msg.update(tmp_state);
    cmd_msg.update(tmp_cmd);

    auto bounds =
        adjust_limits_interval(.02, robot.info().q_min, robot.info().q_max);
    q_min = bounds.first;
    q_max = bounds.second;

    tau_min.resize(7);
    tau_max.resize(7);

    for (size_t i = 0; i < 7; i++) {
      tau_min[i] = -30.;
      tau_max[i] = 30.;
    }
  }

  void switch_mode_sync(flexiv::rdk::Mode t_robot_mode) {
    robot.SwitchMode(t_robot_mode);
    robot_mode = t_robot_mode;
  }

  virtual void get_state(FlexivMsg::FlexivState &state) override {
    state_msg.get(state);
  }

  virtual void send_cmd(const FlexivMsg::FlexivCmd &cmd) override {
    cmd_msg.update(cmd);
  }

  virtual void get_cmd(FlexivMsg::FlexivCmd &cmd) override { cmd_msg.get(cmd); }

private:
  SyncData<FlexivMsg::FlexivCmd> cmd_msg;
  SyncData<FlexivMsg::FlexivState> state_msg;
  std::vector<double> q_min;
  std::vector<double> q_max;

  std::vector<double> tau_min;
  std::vector<double> tau_max;

  FlexivMsg::FlexivCmd
      _cmd; // just to avoid memory allocation. Do not use outside callback!
  FlexivMsg::FlexivState
      _state; // just to avoid memory allocation. Do not use outside callback!

  double max_norm_vel = 6;
  double max_norm_acc = 2;
  double max_distance = .3;
  double max_distance_vel = 7;

  virtual void _send_cmd_torque(const FlexivMsg::FlexivCmd &cmd) {

    // TODO: Preallocate the vectors!
    std::vector<double> torque(7, 0);
    std::vector<double> q = robot.states().q;
    std::vector<double> dq = robot.states().dtheta;

    std::vector<double> target_pose(cmd.q().begin(), cmd.q().end());
    std::vector<double> target_vel(cmd.dq().begin(), cmd.dq().end());

    // check that q is close the current position
    if (euclidean_distance(target_pose, robot.states().q) > max_distance ||
        inf_distance(q, robot.states().q) > max_distance / 2.) {
      std::cout << "ERROR" << std::endl;
      print_vector(target_pose);
      print_vector(robot.states().q);
      throw_pretty("Position is too far from current position");
    }

    if (euclidean_norm(target_vel) > max_norm_vel ||
        inf_norm(target_vel) > max_norm_vel / 2.) {
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

    // check that the desired velocity is close to 0.

    // check that there are non NAns or infs

    for (auto &i : torque) {
      if (std::isnan(i) || std::isinf(i)) {
        throw_pretty("Nan in torque");
      }
    }

    for (size_t i = 0; i < torque.size(); i++) {
      if (torque[i] < tau_min[i] or torque[i] > tau_max[i]) {
        print_vector(torque);
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
      robot.StreamJointTorque(torque);
    }

    // check that the torque is inside limits
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

    // Lots of sanity checks!!

    if (q_min.size() != target_pos.size() ||
        q_max.size() != target_pos.size()) {
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
        throw_pretty("Nan in target velocity");
      }
    }

    for (auto &i : target_acc) {
      if (std::isnan(i) or std::isinf(i)) {
        throw_pretty("Nan in target acceleration");
      }
    }

    if (euclidean_norm(target_vel) > max_norm_vel) {
      throw_pretty("Velocity norm is too high");
    }

    if (euclidean_norm(target_acc) > max_norm_acc) {
      throw_pretty("Acceleration norm is too high");
    }

    if (euclidean_distance(target_pos, robot.states().q) > max_distance ||
        inf_distance(target_pos, robot.states().q) > max_distance / 2.) {
      print_vector(target_pos);
      print_vector(robot.states().q);
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

  virtual void _get_state(FlexivMsg::FlexivState &state) {
    std::string time = get_current_timestamp();
    state.timestamp(time);

    auto q = robot.states().q;
    auto dq = robot.states().dtheta;

    if (q.size() == state.q().size()) {
      std::copy(q.begin(), q.end(), state.q().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }
    if (dq.size() == state.dq().size()) {
      std::copy(dq.begin(), dq.end(), state.dq().begin());
    } else {
      throw_pretty("Vector size does not match array size!");
    }
  }
};

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

    double dt = .001;

    double stop_dt_if_no_msg_ms = 100;
    double max_time_s = 50;
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
    FlexivMsg::FlexivState state_msg, state_last_good_cmd;
    std::vector<double> last_good_state;

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
      bool go_home = true;

      robot = std::make_shared<RealRobot>(robot_serial_number, t_robot_mode,
                                          go_home);
    }

    robot->get_state(state_msg);
    writer.write(state_msg);
    robot->get_cmd(cmd_msg);
    state_last_good_cmd = state_msg;
    robot->send_cmd(cmd_msg);

    std::cout << "First state msg is \n" << state_msg << std::endl;
    std::cout << "First cmd msg is \n" << cmd_msg << std::endl;

    auto tic = std::chrono::high_resolution_clock::now();
    auto last_msg_time = std::chrono::high_resolution_clock::now();
    bool warning_send = false;
    bool good_msg_send = false;
    std::vector<double> time_loop_us_vec;

    robot->start();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - tic)
                   .count() /
               1000. <
           max_time_s) {

      auto tic_loop = std::chrono::high_resolution_clock::now();

      if (std::chrono::duration_cast<std::chrono::milliseconds>(tic_loop -
                                                                last_msg_time)
              .count() > stop_dt_if_no_msg_ms) {

        if (operation_mode == 2) {
          robot->compute_default_cmd(state_last_good_cmd, cmd_msg);
        } else if (operation_mode == 1) {
          robot->compute_default_cmd(state_msg, cmd_msg);
        }

        if (!warning_send) {
          std::cout << "No message received in " << stop_dt_if_no_msg_ms
                    << " ms. Computing a default message" << std::endl;
          warning_send = true;
          good_msg_send = false;
        }
      }

      /* For this example, the reader will return a set of messages (aka
       * Samples). There are other ways of getting samples from reader.
       * See the various read() and take() functions that are present. */
      dds::sub::LoanedSamples<FlexivMsg::FlexivCmd> samples;

      /* Try taking samples from the reader. */
      samples = reader.take();

      if (samples.length() > 0) {
        /* Use an iterator to run over the set of samples. */
        dds::sub::LoanedSamples<FlexivMsg::FlexivCmd>::const_iterator
            sample_iter;
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
            cmd_msg = msg;
            state_last_good_cmd = state_msg; // save the last good state!
            last_msg_time = std::chrono::high_resolution_clock::now();
            if (!good_msg_send) {
              std::cout << "Good message received. Sending it to the robot"
                        << std::endl;
              good_msg_send = true;
              warning_send = false;
            }
          }
        }
      }

      robot->send_cmd(cmd_msg);
      robot->get_state(state_msg);
      writer.write(state_msg);

      int time_loop_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now() - tic_loop)
              .count();

      time_loop_us_vec.push_back(time_loop_us);
      if (time_loop_us > 1e6 * dt) {
        std::cout << "Loop took too long: " << time_loop_us << " us"
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
