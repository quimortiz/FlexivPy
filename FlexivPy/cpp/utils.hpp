#include "FlexivData.hpp"
#include "dds/dds.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <cmath>

// Helper function to format the error message with file, line, and function
// details
std::string format_error_message(const std::string &message, const char *file,
                                 int line, const char *func);

std::pair<std::vector<double>, std::vector<double>>
adjust_limits_interval(double margin, const std::vector<double> &q_min,
                       const std::vector<double> &q_max);

void scale_vector(std::vector<double> &vec, double scale);

// Macro to throw an exception with detailed information
#define throw_pretty(message)                                                  \
  throw std::runtime_error(                                                    \
      format_error_message(message, __FILE__, __LINE__, __PRETTY_FUNCTION__))

std::string get_current_timestamp();

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

template <typename T> double euclidean_norm(const T &vec) {
  double norm = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    norm += vec[i] * vec[i];
  }
  return std::sqrt(norm);
}

template <typename T> double inf_norm(const T &vec) {
  double norm = 0;
  for (auto &v : vec) {
    norm = std::max(norm, std::abs(v));
  }
  return norm;
}


template <typename T, typename U> double euclidean_distance(const T &vec1, const U &vec2) {
  double dist = 0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
  }
  return std::sqrt(dist);
}

template <typename T, typename U> double inf_distance(const T &vec1, const U &vec2) {
  double dist = 0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist = std::max(dist, std::abs(vec1[i] - vec2[i]));
  }
  return dist;
}

Eigen::VectorXd create_vector_from_list(const std::initializer_list<double>& values);




void print_vector(const std::vector<double> &vec);

std::optional<FlexivMsg::FlexivCmd>
read_last_cmd(dds::sub::DataReader<FlexivMsg::FlexivCmd> &reader);
