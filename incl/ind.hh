// Data indexing struct for real-space point voxelization

#ifndef ind_hh
#define ind_hh

#include <iostream>
#include <vector>

#include <sparsepp/spp.h>

#include "occ_data.hh"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct ind
{
  int x, y, z;

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Equality comparator overload
  bool operator==(const ind& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Simple Coutput function for easier debugging
  void print() const { std::cout << this->x << ", "
                                 << this->y << ", "
                                 << this->z << '\n'; }

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // FOR AMR (BEGIN)
  // Fills a vector<ind> of size=8 with child indicies of parent voxel
  void get_child_inds(std::vector<ind> & children,
                      int extent
                      )
  {
    // Iterate through 2x2x2 of children
    for(int z_i = 0; z_i < 2; ++z_i)
    {
      for(int y_i = 0; y_i < 2; ++y_i)
      {
        for(int x_i = 0; x_i < 2; ++x_i)
        {
          // Add children sequentially
          children.push_back( { this->x + (x_i * (extent)),
                                this->y + (y_i * (extent)),
                                this->z + (z_i * (extent)) } );
        }
      }
    }
  }

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Returns an ind structure of parent voxel of input child
  ind get_parent_ind(int extent
                     )
  {
    // Truncate to parent index
    ind ind_parent = { (this->x - (this->x % extent)),
                       (this->y - (this->y % extent)),
                       (this->z - (this->z % extent)) };
    return ind_parent;
  }

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Query whether given index of sparse hash map is collapseable
  bool pruneable(spp::sparse_hash_map<ind, occ_data> & vox,
                 std::vector<ind> & node
                 )
  {
    // Truncate to parent of current node index
    ind cind_corn = { this->x - (this->x % 2),
                      this->y - (this->y % 2),
                      this->z - (this->z % 2) };

    // Iterate through 2x2x2 of children of parent
    for(int z_i = 0; z_i < 2; ++z_i)
    {
      for(int y_i = 0; y_i < 2; ++y_i)
      {
        for(int x_i = 0; x_i < 2; ++x_i)
        {
          // Write children to node vector
          node.push_back( { cind_corn.x + x_i,
                            cind_corn.y + y_i,
                            cind_corn.z + z_i } );
          // Terminate if written node doesn't exist
          if(vox.count( node[ (z_i + 1) * (y_i + 1) * (z_i + 1) - 1 ] ) == 0) { return false; }
        }
      }
    }
    // Debugging
    if(node.size() != 8) { std::cout << node.size() << '\n'; }

    return true;
  }

  // FOR AMR (END)

};

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
