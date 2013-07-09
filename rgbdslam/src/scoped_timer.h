#include <ctime>
class ScopedTimer {
  public:
    ScopedTimer(const char* name);
    ///Log time since construction (if > min_time_reported param)
    ~ScopedTimer();
    ///Get time since construction
    double elapsed();
  private:
    struct timespec start;
    const char* name;
};
