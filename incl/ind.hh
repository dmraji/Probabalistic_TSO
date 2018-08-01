// Data indexing struct for real-space point voxelization

#ifndef ind_hh
#define ind_hh

// #include "multi_dim_array.hh"

// Boost Libraries
#include <boost/functional/hash.hpp>

struct ind
{
  int x, y, z;

  // Equality comparator overload
  bool operator==(const ind& other
                  ) const
  {
    return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
  }

  // Fills a vector<ind> of size=8 with child indicies of parent voxel
  void get_child_inds(std::vector<ind> & children,
                      int extent
                      )
  {
    for(int z_i = 0; z_i < 2; ++z_i)
    {
      for(int y_i = 0; y_i < 2; ++y_i)
      {
        for(int x_i = 0; x_i < 2; ++x_i)
        {
          children.push_back( { this->x + (x_i * (extent)),
                                this->y + (y_i * (extent)),
                                this->z + (z_i * (extent)) } );
        }
      }
    }
  }

  // Returns an ind structure of parent voxel of input child
  ind get_parent_ind(int extent
                     )
  {
    ind ind_parent = { (this->x - (this->x % extent)),
                       (this->y - (this->y % extent)),
                       (this->z - (this->z % extent)) };
    return ind_parent;
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
