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

  virtual void start() {}
  virtual void stop() {}
  virtual void get_cmd(FlexivMsg::FlexivCmd &cmd) {}

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

double euclid_norm(const std::vector<double> &vec) {
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

  // this is called by the scheduler
  void callback() {
    cmd_msg.get(_cmd);
    _send_cmd(_cmd);
    _get_state(_state);
    state_msg.update(_state);
  }

  virtual void start() override {
    scheduler.AddTask([&] { this->callback(); }, "General Task", 1,
                      scheduler.max_priority());
    scheduler.Start();
  }

  virtual void stop() override { scheduler.Stop(); }

  RealRobot(std::string robot_serial_number, bool go_home = true)
      : robot(robot_serial_number) {
    std::cout << "Real robot created" << std::endl;

    // Clear fault on the connected robot if any
    if (robot.fault()) {
      spdlog::warn(
          "Fault occurred on the connected robot, trying to clear ...");
      // Try to clear the fault
      if (!robot.ClearFault()) {
        spdlog::error("Fault cannot be cleared, exiting ...");
        throw std::runtime_error("Fault cannot be cleared");
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
      robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
      robot.ExecutePrimitive("Home()");

      // Wait for the primitive to finish
      while (robot.busy()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // Real-time Joint Position Control
    // =========================================================================================
    // Switch to real-time joint position control mode
    robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_POSITION);

    // Set initial joint positions
    auto init_pos = robot.states().q;
    spdlog::info("Initial joint positions set to: {}",
                 flexiv::rdk::utility::Vec2Str(init_pos));

    FlexivMsg::FlexivState tmp_state;
    FlexivMsg::FlexivCmd tmp_cmd;

    _get_state(tmp_state);
    tmp_cmd.q() = tmp_state.q();

    std::cout << "Initial state is " << std::endl;
    std::cout << tmp_state << std::endl;

    std::cout << "Initial cmd is" << std::endl;
    std::cout << tmp_cmd << std::endl;

    state_msg.update(tmp_state);
    cmd_msg.update(tmp_cmd);
  }

  virtual void get_state(FlexivMsg::FlexivState &state) override {
    state_msg.get(state);
  }

  virtual void send_cmd(const FlexivMsg::FlexivCmd &cmd) override {
    cmd_msg.update(cmd);
  }

  virtual void get_cmd(FlexivMsg::FlexivCmd &cmd) { cmd_msg.get(cmd); }

private:
  SyncData<FlexivMsg::FlexivCmd> cmd_msg;
  SyncData<FlexivMsg::FlexivState> state_msg;

  FlexivMsg::FlexivCmd
      _cmd; // just to avoid memory allocation. Do not use outside callback!
  FlexivMsg::FlexivState
      _state; // just to avoid memory allocation. Do not use outside callback!

  double max_norm_vel = 6;
  double max_norm_acc = 2;
  double max_distance = .3;

  virtual void _send_cmd(const FlexivMsg::FlexivCmd &cmd) {

    std::vector<double> target_pos(7, 0);
    std::vector<double> target_vel(7, 0);
    std::vector<double> target_acc(7, 0);

    if (cmd.q().size() == target_pos.size()) {
      std::copy(cmd.q().begin(), cmd.q().end(), target_pos.begin());
    } else {
      std::cerr << "Vector size does not match array size!" << std::endl;
      throw std::runtime_error("Vector size does not match array size!");
    }
    /*std::cout << "Sending command to robot" << std::endl;*/
    /*std::cout << get_current_timestamp() << std::endl;*/

    // check that the euclid
    if (euclid_norm(target_vel) > max_norm_vel) {
      throw std::runtime_error("Velocity norm is too high");
    }

    if (euclid_norm(target_acc) > max_norm_acc) {
      throw std::runtime_error("Acceleration norm is too high");
    }

    if (euclidean_distance(target_pos, robot.states().q) > max_distance) {
      throw std::runtime_error("Position is too far from current position");
    }

    /*std::cout << "Sending command to robot" << std::endl;*/
    /**/
    /*print_vector(target_pos);*/
    /*print_vector(robot.states().q);*/
    /**/
    /*print_vector(target_vel);*/
    /*print_vector(target_acc);*/
    /**/
    robot.StreamJointPosition(target_pos, target_vel, target_acc);
  }

  virtual void _get_state(FlexivMsg::FlexivState &state) {
    std::string time = get_current_timestamp();
    state.timestamp(time);

    auto q = robot.states().q;
    auto dq = robot.states().dq;

    if (q.size() == state.q().size()) {
      std::copy(q.begin(), q.end(), state.q().begin());
    } else {
      throw std::runtime_error("Vector size does not match array size!");
    }
    if (dq.size() == state.dq().size()) {
      std::copy(dq.begin(), dq.end(), state.dq().begin());
    } else {
      throw std::runtime_error("Vector size does not match array size!");
    }
  }
};

int main(int argc, char *argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " <{0:fake,1:real}> <robot_serial_number>" << std::endl;
    return 1; // Return a non-zero value to indicate an error
  }

  // Extract the robot serial number from the command-line arguments
  int mode = std::stoi(argv[1]);
  std::string robot_serial_number = argv[2];

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

    /* Now, the reader can be created to subscribe to a HelloWorld message. */
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

    std::shared_ptr<Robot> robot;

    if (mode == 0) {
      bool write_to_file = false;
      robot = std::make_shared<Fake_robot>(write_to_file);
    } else if (mode == 1) {
      robot = std::make_shared<RealRobot>(robot_serial_number);
    }

    robot->get_state(state_msg);
    writer.write(state_msg);
    robot->get_cmd(cmd_msg);
    std::cout << "First state msg is " << std::endl;
    std::cout << state_msg << std::endl;

    std::cout << "First cmd msg is" << std::endl;
    std::cout << cmd_msg << std::endl;

    robot->send_cmd(cmd_msg);

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

        cmd_msg.q() =
            state_msg.q(); // NOTE: this might drift! -- try to put in loop
        std::fill(cmd_msg.dq().begin(), cmd_msg.dq().end(), 0.0);

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
      /*robot->get_state(state_msg);*/
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
