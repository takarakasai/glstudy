
#include <cstdio>
#include <ctime>
#include <stdint.h>

#ifdef __APPLE__
#include <sys/time.h>

#define CLOCK_REALTIME 0

//struct timespec {
//  time_t tv_sec; /* seconds */
//  long tv_nsec;  /* nano seconds */
//};

typedef int clockid_t;
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
  struct timeval tim;

  gettimeofday(&tim, NULL);

  tp->tv_sec  = tim.tv_sec;
  tp->tv_nsec = tim.tv_usec * 1000;

  return 0;
}
#endif

class cycle_measure {
private:
  struct timespec before_;
  uint16_t period_;
  uint16_t count_;
  bool is_cout_;

public:
  cycle_measure(uint16_t period) {
    clock_gettime(CLOCK_REALTIME, &before_);
    period_ = period;
    count_ = 0;
    is_cout_ = false;
    return;
  }

  virtual ~cycle_measure() {
    return;
  }

  void set_cout (bool is_cout) {
    is_cout_ = is_cout;
    return;
  }

  void update() {
    if (++count_ == period_) {
      struct timespec cur;
      double diff;
      clock_gettime(CLOCK_REALTIME, &cur);

      if (cur.tv_nsec < before_.tv_nsec) {
        diff  = 1000*1000*1000 * (cur.tv_sec - before_.tv_sec - 1);
        diff += 1000*1000*1000 + cur.tv_nsec - before_.tv_nsec;
      } else {
        diff  = 1000*1000*1000 * (cur.tv_sec - before_.tv_sec);
        diff +=                  cur.tv_nsec - before_.tv_nsec;
      }

      memcpy(&before_, &cur, sizeof(cur));

      double spf = diff / period_ / 1000 / 1000 / 1000;
      double fps = 1.0 / spf;

      if (is_cout_) {
        printf("%9.2lf[fps] (%3.4lf[sec])\n", fps, spf);
      }
 
      count_ = 0;
    }
  }
};

