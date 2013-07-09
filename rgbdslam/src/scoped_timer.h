#include <ctime>
class ScopedTimer {
  public:
    ///Log time elapsed at destruction (if > min_time_reported param or unconditional_logging = true)
    ScopedTimer(const char* name, bool unconditional_logging = false);
    ///Log time since construction (if > min_time_reported param)
    ~ScopedTimer();
    ///Get time since construction
    double elapsed();
  private:
    struct timespec start;
    const char* name;
    bool unconditional_triggering;
};
