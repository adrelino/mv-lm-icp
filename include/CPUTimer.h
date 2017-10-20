#ifndef CPU_TIMER_H
#define CPU_TIMER_H

#include <iostream>
#include <string>
#include <map>
#include <chrono>

typedef std::chrono::steady_clock Clock;

class CPUTimer {
 private:
  Clock::time_point startTime;
  std::map<std::string,float> timingsMap;

 public:
  void tic();
  void toc(std::string name);
  void printAllTimings();
};
#endif
