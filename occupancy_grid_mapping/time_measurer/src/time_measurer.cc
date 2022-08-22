#include "time_measurer/time_measurer.h"

namespace time_measurer {

double ToSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

TimeMeasurer::TimeMeasurer(std::string name="Time measurer", bool print_results_on_destruction=false) {
  std::lock_guard<std::mutex> lock(mutex_);
  name_ = name;
  print_results_on_destruction_ = print_results_on_destruction;
}

void TimeMeasurer::StartMeasurement() {
  std::lock_guard<std::mutex> lock(mutex_);
  start_time_[pthread_self()] = std::chrono::steady_clock::now();
}

void TimeMeasurer::StopMeasurement() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto stop_time = std::chrono::steady_clock::now();
  double measured_time = ToSeconds(stop_time - start_time_[pthread_self()]);
  time_measurements_.push_back(measured_time);
}

void TimeMeasurer::AddMeasurement(double measured_time) {
  std::lock_guard<std::mutex> lock(mutex_);
  start_time_[pthread_self()];
  time_measurements_.push_back(measured_time);
}

TimeMeasurer::~TimeMeasurer() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (print_results_on_destruction_ && time_measurements_.size()) {
    double total_measured_time = 0.;
    double avarage_time = 0.;
    double max_time = 0;
    for (auto measured_time : time_measurements_) {
      total_measured_time += measured_time;
      max_time = max_time > measured_time ? max_time : measured_time;
    }
    avarage_time = total_measured_time / time_measurements_.size();

    std::string log_string;
    log_string += name_ + ":\n";
    log_string += "    Number of measurements: " + std::to_string(time_measurements_.size()) + "\n";
    log_string += "    Total measured time: " + std::to_string(total_measured_time) + "\n";
    log_string += "    Average time: " + std::to_string(avarage_time) + "\n";
    log_string += "    Max time: " + std::to_string(max_time) + "\n";
    log_string += "    Number of threads: " + std::to_string(start_time_.size()) + "\n";

    std::cout << log_string;
  }
}

}  // namespace time_measurer
