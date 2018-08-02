// Data indexing struct for real-space point voxelization

#ifndef ind_hh
#define ind_hh

#include <sparsepp/spp.h>

struct ind
{
  const int x, y, z;

  // Equality comparator overload
  bool operator==(const ind& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  // Fills a 2x2x2 array with child indecies of parent voxel
  void get_child_inds(const ind& parent,
                      std::vector<ind> & children,
                      int pr_extent
                      ) const
  {
    for(int z_i = 0; z_i < 2; ++z_i)
    {
      for(int y_i = 0; y_i < 2; ++y_i)
      {
        for(int x_i = 0; x_i < 2; ++x_i)
        {
          children.push_back( { parent.x + (x_i * (pr_extent / 2)),
                                parent.y + (y_i * (pr_extent / 2)),
                                parent.z + (z_i * (pr_extent / 2)) } );
        }
      }
    }
  }

  // Returns an ind structure of parent voxel of input child
  ind get_parent_ind(const ind& child,
                     const ind& min,
                     int pr_extent
                     ) const
  {
    ind parent = { (child.x - (child.x % pr_extent) + (min.x % 2)),
                   (child.y - (child.y % pr_extent) + (min.y % 2)),
                   (child.z - (child.z % pr_extent) + (min.z % 2)) };
    return parent;
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
        spp::hash_combine(seed, index.x);
        spp::hash_combine(seed, index.y);
        spp::hash_combine(seed, index.z);
        return seed;
      }
    };
}

#endif
