// Find bounding box of pointcloud (header)

#ifndef bbx_hh
#define bbx_hh

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>

// Sparse hash map
#include <sparsepp/spp.h>

#include "ind.hh"
#include "occ_data.hh"

struct corners
{
  int min_x, min_y, min_z, max_x, max_y, max_z;
  float mprob;

  corners(float mean_probability) : mprob(mean_probability) {}

  // Bounding box constructor
  void get_corners(spp::sparse_hash_map<ind, occ_data> & occ,
                   float thresh
                   )
  {
    std::cout << thresh * mprob << '\n';
    min_x = occ.begin()->first.x;
    max_x = occ.begin()->first.x;
    min_y = occ.begin()->first.y;
    max_y = occ.begin()->first.y;
    min_z = occ.begin()->first.z;
    max_z = occ.begin()->first.z;

    spp::sparse_hash_map <ind, occ_data> ::iterator it;
    for(it = occ.begin(); it != occ.end(); ++it)
    {
      // if(occ[ {it->first.x, it->first.y, it->first.z} ].mask)
      // {
        ind cind = {it->first.x, it->first.y, it->first.z};
        if(occ[cind].probability > (thresh * mprob))
        {
          if(cind.x < this->min_x) { this->min_x = cind.x; } else if (cind.x > this->max_x) { this->max_x = cind.x; }
          if(cind.y < this->min_y) { this->min_y = cind.y; } else if (cind.y > this->max_y) { this->max_y = cind.y; }
          if(cind.z < this->min_z) { this->min_z = cind.z; } else if (cind.z > this->max_z) { this->max_z = cind.z; }
        }
      // }
    }
  }

  // Ensure even extent in each axis of bounding box
  void even_out()
  {
    if((max_x % 2) == 1) { ++this->max_x; }
    if((min_x % 2) == 1) { --this->min_x; }
    if((max_y % 2) == 1) { ++this->max_y; }
    if((min_y % 2) == 1) { --this->min_y; }
    if((max_z % 2) == 1) { ++this->max_z; }
    if((min_z % 2) == 1) { --this->min_z; }
    std::cout << "x: " << this->min_x << " " << this->max_x << '\n';
    std::cout << "y: " << this->min_y << " " << this->max_y << '\n';
    std::cout << "z: " << this->min_z << " " << this->max_z << '\n';
  }

};

#endif
