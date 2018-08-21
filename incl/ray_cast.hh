// Ray-casting non-member (header)

#ifndef ray_cast_hh
#define ray_cast_hh

#include <iostream>

#include <vector>

#include <cmath>

#include "pt.hh"

#include "il_math.hh"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void cast_ray(pt &origin,
              pt &end,
              float resolution,
              std::vector<pt> & ray
              );

#endif
