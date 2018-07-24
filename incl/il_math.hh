// Simple math inlines

#ifndef il_math_hh
#define il_math_hh

inline float sq(float n) { return (float)n * (float)n; }

inline int f_floor(float n) { return (int)(n + 32768.) - 32768; }

inline bool is_zero(float n) { return n == 0.0f; }

#endif
