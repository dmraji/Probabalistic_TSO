// Simple math inlines

#ifndef il_math_hh
#define il_math_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

inline float sq(float n) { return (float)n * (float)n; }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

inline int f_floor(float n) { return (int)(n + 32768.) - 32768; }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

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
