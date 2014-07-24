#ifndef RGBDSLAM_SCOPED_TIMER_H
#define RGBDSLAM_SCOPED_TIMER_H
#include <ctime>

///Class that outputs the time since construction on destruction.
///Depends on the loggerlevel. If ROSCONSOLE_MIN_SEVERITY is set
///above INFO, only Timers with unconditional_logging=true will
///be active (and output on WARN level).
class ScopedTimer {
  public:
    ///Log time elapsed at destruction (if > min_time_reported param or unconditional_logging = true)
    ScopedTimer(const char* name, bool only_for_logging = true, bool unconditional_logging = false);
    ///Log time since construction (if > min_time_reported param)
    ~ScopedTimer();
    ///Get time since construction
    double elapsed();
  private:
    struct timespec start;
    const char* name;
    bool unconditional_triggering;
};

#endif
