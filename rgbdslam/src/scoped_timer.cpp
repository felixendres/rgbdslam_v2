#include "scoped_timer.h"
#include <ros/console.h>
#include "parameter_server.h"

ScopedTimer::ScopedTimer(const char* thename, bool unconditional_logging){
  name = thename;
  unconditional_triggering = unconditional_logging;
  clock_gettime(CLOCK_MONOTONIC, &start);
}

double ScopedTimer::elapsed(){
    struct timespec now; 
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
}

ScopedTimer::~ScopedTimer(){
  double min_time = ParameterServer::instance()->get<double>("min_time_reported");
  if(unconditional_triggering || min_time > 0){
    double runtime = elapsed();
    if(unconditional_triggering || runtime > min_time){
      ROS_INFO_STREAM_NAMED("timings", name << " runtime: "<< runtime <<" s");
    }
  }
}

