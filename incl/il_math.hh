// Simple math inlines

#ifndef il_math_hh
#define il_math_hh

#include <cassert>

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Quick square function
inline float sq(float n) { return (float)n * (float)n; }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Quick floor function (truncate towards zero)
inline int f_floor(float n)
{
  assert(n >= 0.0f);
  return (int)(n + 32768.) - 32768;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Quick zero-query
inline bool is_zero(float n) { return n == 0.0f; }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// inline int modulo(int n,
//                   int m
//                   )
// {
//   const unsigned int s;
//   const unsigned int d = 1U << s;
//   unsigned int m;
//   return n & (d - 1);
// }

#endif
