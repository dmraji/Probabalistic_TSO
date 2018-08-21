// Ray-casting non-member (source)

#include "ray_cast.hh"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void cast_ray(pt &origin,
              pt &end,
              float resolution,
              std::vector<pt> & ray
              )
{
  // Calculate Euclidean distance between origin (current pose) and end (point in cloud)
  pt dist = {end.x-origin.x, end.y-origin.y, end.z-origin.z};

  // Calculate magnitude of Euclidean distance
  float mag = std::sqrt(sq(dist.x) + sq(dist.y) + sq(dist.z));

  // Discretize magnitude by resolution of voxels
  int disc = int(std::ceil(mag/resolution));

  // Define size of increment along path of ray
  pt inc = {dist.x / disc, dist.y / disc, dist.z / disc};

  // Warn on short ray
  if(disc < 1)
  {
    std::cout << "WARNING: Ray of length less than 1!" << std::endl;
  }

  // Do not mark end (point in cloud) as free
  ray.reserve(disc-1);

  // Iterate along discretized ray
  for(int i = 0; i < disc-1; ++i)
  {
    // Add points to ray vector
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i, origin.z + inc.z * i});
  }

}
