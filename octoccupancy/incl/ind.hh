// Data indexing struct for real-space point voxelization

#ifndef ind_hh
#define ind_hh

#include <sparsepp/spp.h>

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct ind
{
  const int x, y, z;

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Equality comparator overload
  bool operator==(const ind& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Hash template for indexing
namespace std
{
  template<>
    struct hash<ind>
    {
      size_t operator()(const ind& index
                        ) const
      {
        size_t seed = 0;
        spp::hash_combine(seed, index.x);
        spp::hash_combine(seed, index.y);
        spp::hash_combine(seed, index.z);
        return seed;
      }
    };
}

#endif
