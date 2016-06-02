
#ifndef DP_ERR
#define DP_ERR

#define ECALL(function)     \
  do {                      \
    errno_t eno = function; \
    if (eno != 0) {         \
      return eno;           \
    }                       \
  } while(0)

namespace Dp {
  typedef int errno_t;
}

#endif

