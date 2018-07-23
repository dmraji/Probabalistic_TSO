// Data indexing struct for real-space point voxelization

#ifndef ind_hh
#define ind_hh

#include "multi_dim_array_hh"

// Boost Libraries
#include <boost/functional/hash.hpp>

struct ind
{
  const int x, y, z;

  ind() : x(0),
          y(0),
          z(0)
          {}

  // Equality comparator overload
  bool operator==(const ind& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  // Fills a 2x2x2 array with child indecies of parent voxel
  void get_child_inds(const ind& parent,
                      md_array<ind, 2, 2, 2> & children,
                      int rel_extent
                      ) const
  {
    for(int x_i = 0; x_i < 2; ++x_i)
    {
      for(int y_i = 0; y_i < 2; ++y_i)
      {
        for(int z_i = 0; z_i < 2; ++z_i)
        {
          children[x_i][y_i][z_i] = {parent.x + (x_i / rel_extent),
                                     parent.y + (y_i / rel_extent),
                                     parent.z + (z_i / rel_extent)};
        }
      }
    }
  }
  
};

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
        boost::hash_combine(seed, index.x);
        boost::hash_combine(seed, index.y);
        boost::hash_combine(seed, index.z);
        return seed;
      }
    };
}

#endif
