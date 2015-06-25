#ifndef CPU_TIMER_H
#define CPU_TIMER_H

#include <iostream>
#include <string>
#include <map>
#include <time.h>

#ifndef CLOCK_REALTIME
  #define CLOCK_REALTIME CLOCK_MONOTONIC
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef _WIN32
#include <wintime.h>
#else
#include <sys/time.h>
#endif

class CPUTimer {
 private:
  timespec startTime;
  timespec endTime;

  bool startWasSet;
  bool stopWasSet;

  void getTime(struct timespec *ts);

  std::map<std::string,float> timingsMap;

 public:
  CPUTimer();
  ~CPUTimer();

  void tic();
  timespec toc();
  void toc(std::string name);

  float tocSeconds();

  void printAllTimings();

  std::string getHeader();
  std::string getMeasurements();
};
#endif
