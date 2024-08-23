#include <Eigen/Core>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <chrono>


#include "utils.hpp"
#include <iostream>

#include "FlexivData.hpp"

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




void print_vector(const std::vector<double> &vec) {
  for (auto &v : vec) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
}


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

Eigen::VectorXd create_vector_from_list(const std::initializer_list<double>& values) {
    Eigen::VectorXd vec(values.size());
    int index = 0;
    for (const auto& value : values) {
        vec(index++) = value;
    }
    return vec;
}


