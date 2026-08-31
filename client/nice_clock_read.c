/**
 * @file nice_clock_read.c
 * @brief Prints the time of the bus one time, and gives 0 while that time is available.
 *
 * @code
 * cc -o nice_clock_read client/nice_clock_read.c && ./nice_clock_read
 * @endcode
 */
#include <stdio.h>

#include "nice_clock.h"

int main(void) {
  int64_t t_DC_ns = 0;
  int64_t t_PC_ns = 0;
  int64_t age_ns = 0;

  if (nice_clock_now(&t_DC_ns, &t_PC_ns, &age_ns) != 0) {
    return 1;  // nice_clock_now() reported the reason
  }
  printf("t_DC %lld ns, t_PC %lld ns, from a record %.3f s old\n", (long long)t_DC_ns, (long long)t_PC_ns,
         1e-9 * (double)age_ns);
  return 0;
}
