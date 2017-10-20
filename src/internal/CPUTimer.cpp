#include <cassert>
#include <CPUTimer.h>

#include <stdio.h>
#include <iomanip> 

#include <iostream>
#include <sstream>

using namespace std;

void CPUTimer::tic() {
  startTime = Clock::now();
}

void CPUTimer::toc(std::string name){
  Clock::time_point endTime = Clock::now();
  auto s = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  double seconds = s.count() * 1e-6f; //1e6 microseconds is 1s
  cout<<endl;
  cout<<"=====  TIMING["<<name<<"] is ";
  cout<< seconds << " s" << endl;          
  cout << endl;

  timingsMap[name] = seconds;
}

void CPUTimer::printAllTimings(){
  typedef std::map<std::string,float>::iterator it_type;
    cout<<"=====  TIMINGS ===="<<endl;

  for(it_type it = timingsMap.begin(); it != timingsMap.end(); ++it) {
    cout<< left << setw(20) << it->first<<":\t";
    printf("%0.3f\n",it->second);
  }
}
