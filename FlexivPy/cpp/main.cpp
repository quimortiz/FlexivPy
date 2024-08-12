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

class Fake_robot {

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

  void get_state(Flexiv::FlexivState &state) {
    std::string time = get_current_timestamp();
    state.timestamp(time);

    if (write_to_file) {
      log_file_state << "**" << std::endl;
      log_file_state << "Getting state from robot at " << time << std::endl;
      log_file_state << "Message has time stamp: " << time << std::endl;
    }
  }

  void send_cmd(const Flexiv::FlexivCmd &cmd) {
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

int main() {
  // TODO: do a fake robot that reads/write on the topics :) -- add timings
  // Would it make sense to write the simulator in c++?
  // What to do with image, etc?
  // What is the correct way for users to use the code?
  // nice :)
  //
  try {

    // letflexiv::rdk::Robot robot(robot_sn);s create a real robot!

    std::string robot_sn = "";
    flexiv::rdk::Robot robot2(robot_sn);

    std::cout << "=== [Subscriber] Create reader." << std::endl;

    double rate = 1000; // 1 kHz
    double dt = 1. / rate;

    bool write_to_file = false;
    Fake_robot robot(write_to_file);

    /* First, a domain participant is needed.
     * Create one on the default domain. */
    dds::domain::DomainParticipant participant(domain::default_id());

    // Set QoS for the Topic
    dds::topic::qos::TopicQos topic_qos =
        participant.default_topic_qos()
        << dds::core::policy::History::KeepLast(30);
    /*<< dds::core::policy::ResourceLimits(1, dds::core::LENGTH_UNLIMITED,*/
    /*                                     dds::core::LENGTH_UNLIMITED);*/

    /* To subscribe to something, a topic is needed. */
    dds::topic::Topic<Flexiv::FlexivCmd> topic_cmd(participant, "FlexivCmd",
                                                   topic_qos);

    /* A reader also needs a subscriber. */
    dds::sub::Subscriber subscriber(participant);

    // Set QoS for the DataReader
    dds::sub::qos::DataReaderQos dr_qos =
        subscriber.default_datareader_qos()
        << dds::core::policy::History::KeepLast(10);
    /*<< dds::core::policy::ResourceLimits(1, dds::core::LENGTH_UNLIMITED,*/
    /*                                     dds::core::LENGTH_UNLIMITED);*/

    /* Now, the reader can be created to subscribe to a HelloWorld message. */
    dds::sub::DataReader<Flexiv::FlexivCmd> reader(subscriber, topic_cmd,
                                                   dr_qos);

    /*  // Create the DataReader with the QoS settings*/
    /*  dds::sub::DataReader<YourMessageType> reader(subscriber, topic,
     * dr_qos);*/
    /**/
    /**/
    /**/
    /*// Set QoS for the DataReader*/
    /*      //*/
    /*      //*/
    /*      //*/
    /*      //*/
    /*  dds::sub::qos::DataReaderQos dr_qos =
     * subscriber.default_datareader_qos()*/
    /*      << dds::core::policy::History::KeepLast(1)*/
    /*      << dds::core::policy::ResourceLimits(1, dds::core::LENGTH_UNLIMITED,
     * dds::core::LENGTH_UNLIMITED);*/

    /* To publish something, a topic is needed. */
    dds::topic::Topic<Flexiv::FlexivState> topic_state(participant,
                                                       "FlexivState");

    /* A writer also needs a publisher. */
    dds::pub::Publisher publisher(participant);

    /* Now, the writer can be created to publish a HelloWorld message. */
    dds::pub::DataWriter<Flexiv::FlexivState> writer(publisher, topic_state);

    /* Poll until a message has been read.
     * It isn't really recommended to do this kind wait in a polling loop.
     * It's done here just to illustrate the easiest way to get data.
     * Please take a look at Listeners and WaitSets for much better
     * solutions, albeit somewhat more elaborate ones. */
    std::cout << "=== [Subscriber] Wait for message." << std::endl;
    bool poll = true;

    double stop_dt_if_no_msg_ms =
        100; // if no message is received in this time
             // we send the default command to the robot.

    Flexiv::FlexivCmd default_msg;
    default_msg.timestamp(get_current_timestamp());

    Flexiv::FlexivCmd latest_msg;
    Flexiv::FlexivState state_msg;

    auto tic = std::chrono::high_resolution_clock::now();
    auto last_msg_time = std::chrono::high_resolution_clock::now();
    double max_time_s = 20;
    bool warning_send = false;
    bool good_msg_send = false;
    std::vector<double> time_loop_us_vec;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - tic)
                   .count() /
               1000. <
           max_time_s) {

      auto tic_loop = std::chrono::high_resolution_clock::now();

      if (std::chrono::duration_cast<std::chrono::milliseconds>(tic_loop -
                                                                last_msg_time)
              .count() > stop_dt_if_no_msg_ms) {
        latest_msg = default_msg;
        if (!warning_send) {
          std::cout << "No message received in " << stop_dt_if_no_msg_ms
                    << " ms. Sending default message" << std::endl;
          warning_send = true;
          good_msg_send = false;
        }
      }

      /* For this example, the reader will return a set of messages (aka
       * Samples). There are other ways of getting samples from reader.
       * See the various read() and take() functions that are present. */
      dds::sub::LoanedSamples<Flexiv::FlexivCmd> samples;

      /* Try taking samples from the reader. */
      samples = reader.take();

      /*if (!samples.length()) {*/
      /*  std::cout << "not samples!" << std::endl;*/
      /*}*/
      /**/
      /*if (samples.length() > 1) {*/
      /*  std::cout << "Samples length: " << samples.length() << std::endl;*/
      /*}*/
      /**/
      /* Are samples read? */
      if (samples.length() > 0) {
        /* Use an iterator to run over the set of samples. */
        dds::sub::LoanedSamples<Flexiv::FlexivCmd>::const_iterator sample_iter;
        for (sample_iter = samples.begin(); sample_iter < samples.end();
             ++sample_iter) {
          /* Get the message and sample information. */
          const Flexiv::FlexivCmd &msg = sample_iter->data();
          const dds::sub::SampleInfo &info = sample_iter->info();

          /* Sometimes a sample is read, only to indicate a data
           * state change (which can be found in the info). If
           * that's the case, only the key value of the sample
           * is set. The other data parts are not.
           * Check if this sample has valid data. */
          if (info.valid()) {
            latest_msg = msg;
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

      robot.send_cmd(latest_msg);
      robot.get_state(state_msg);
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

      /*std::this_thread::sleep_for(*/
      /*    std::chrono::microseconds(std::max(0, 1000 - time_loop_us)));*/

      std::this_thread::sleep_for(std::chrono::microseconds(
          std::max(0, static_cast<int>(1e6 * dt - time_loop_us))));
    }

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
