#include "scoped_timer.h"
#include <ros/console.h>
#include "parameter_server.h"

ScopedTimer::ScopedTimer(const char* thename){
  name = thename;
  clock_gettime(CLOCK_MONOTONIC, &start);
}

double ScopedTimer::elapsed(){
    struct timespec now; 
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
}

ScopedTimer::~ScopedTimer(){
  double min_time = ParameterServer::instance()->get<double>("min_time_reported");
  if(min_time > 0){
    double runtime = elapsed();
    ROS_INFO_STREAM_COND_NAMED(runtime > min_time, "timings", name << " runtime: "<< runtime <<" s");
  }
}

