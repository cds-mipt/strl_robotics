#ifndef TIME_MEASURER_H_
#define TIME_MEASURER_H_

#include <chrono>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <mutex>

namespace time_measurer {

double ToSeconds(std::chrono::steady_clock::duration duration);

class TimeMeasurer {
 public:
  TimeMeasurer(std::string name, bool print_results_on_destruction);
  ~TimeMeasurer();

  void StartMeasurement();
  void StopMeasurement();

  void AddMeasurement(double measured_time);

 private:
  std::mutex mutex_;
  std::string name_;
  bool print_results_on_destruction_;
  std::map<pthread_t, std::chrono::time_point<std::chrono::steady_clock>> start_time_;
  std::vector<double> time_measurements_;
};

#define MEASURE_TIME_FROM_HERE(name) \
  static time_measurer::TimeMeasurer (time_measurer_ ## name)(#name, true); \
  (time_measurer_ ## name).StartMeasurement()

#define STOP_TIME_MEASUREMENT(name) \
  (time_measurer_ ## name).StopMeasurement()

#define MEASURE_BLOCK_TIME(name) \
  static time_measurer::TimeMeasurer (time_measurer_ ## name)(#name, true); \
  class time_measurer_stop_trigger_class_ ## name { \
   public: \
    (time_measurer_stop_trigger_class_ ## name)() {}; \
    (~time_measurer_stop_trigger_class_ ## name)() {(time_measurer_ ## name).StopMeasurement();}; \
  }; \
  time_measurer_stop_trigger_class_ ## name    time_measurer_stop_trigger_ ## name; \
  (time_measurer_ ## name).StartMeasurement()

}  // namespace time_measurer

#endif  // TIME_MEASURER_H_
