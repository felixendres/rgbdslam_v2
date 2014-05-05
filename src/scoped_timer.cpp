#include "scoped_timer.h"
#include <ros/console.h>
#include "parameter_server.h"

ScopedTimer::ScopedTimer(const char* thename, bool only_for_logging, bool unconditional_logging)
 : name(thename), unconditional_triggering(unconditional_logging)
{
#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_INFO)
  if(!only_for_logging) //would not log anything if level above INFO anyway
#endif
  {
    clock_gettime(CLOCK_MONOTONIC, &start);
  }
}

double ScopedTimer::elapsed(){
  struct timespec now; 
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) * 1e-9;
}

ScopedTimer::~ScopedTimer(){
  //No output anyway if above INFO level - ergo do nothing
#if (ROSCONSOLE_MIN_SEVERITY < ROSCONSOLE_SEVERITY_WARN)
  double min_time = ParameterServer::instance()->get<double>("min_time_reported");
  if(unconditional_triggering || min_time > 0){
    double runtime = elapsed();
    if(unconditional_triggering || runtime > min_time){
      ROS_INFO_STREAM_NAMED("timings", name << " runtime: "<< runtime <<" s");
    }
  }
#endif
}

