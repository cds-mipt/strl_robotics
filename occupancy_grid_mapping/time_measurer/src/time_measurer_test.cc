#include "time_measurer/time_measurer.h"
#include <unistd.h>
#include <iostream>

void long_operation(unsigned int n) {
  usleep(n);
}

int main() {
  time_measurer::TimeMeasurer long_operation_time_measurer("long_operation", true);
  int num = 10;
  unsigned int operation_duration = 100000;
  for (int i = 0; i < num; i++) {
    long_operation_time_measurer.StartMeasurement();
    long_operation(operation_duration);
    long_operation_time_measurer.StopMeasurement();
  }
}
