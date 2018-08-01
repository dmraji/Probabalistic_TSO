// Find bounding box of pointcloud (header)

#ifndef bbx_hh
#define bbx_hh

#include <iostream>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>

#include <sparsepp/spp.h>

#include "ind.hh"
#include "occ_data.hh"

struct corners
{
  int min_x, min_y, min_z, max_x, max_y, max_z;

  // Bounding box constructor
  corners(spp::sparse_hash_map<ind, occ_data> & occ,
          float mean_probability
          )
  {
    min_x = occ.begin()->first.x;
    max_x = occ.begin()->first.x;
    min_y = occ.begin()->first.y;
    max_y = occ.begin()->first.y;
    min_z = occ.begin()->first.z;
    max_z = occ.begin()->first.z;

    spp::sparse_hash_map <ind, occ_data> ::iterator it;
    for(it = occ.begin(); it != occ.end(); ++it)
    {
      if(occ[ {it->first.x, it->first.y, it->first.z} ].mask)
      {
        ind cpt = {it->first.x, it->first.y, it->first.z};
        if(occ[cpt].probability > (12.0f * mean_probability))
        {
          int x_v = cpt.x;
          int y_v = cpt.y;
          int z_v = cpt.z;
          if(x_v < min_x) { min_x = x_v; } else if (x_v > max_x) { max_x = x_v; }
          if(y_v < min_y) { min_y = y_v; } else if (y_v > max_y) { max_y = y_v; }
          if(z_v < min_z) { min_z = z_v; } else if (z_v > max_z) { max_z = z_v; }
        }
      }
    }
  }

  // Ensure even extent in each axis of bounding box
  void even_out()
  {
    // if((std::abs(max_x - min_x) % 2) == 1) { ++max_x; }
    // if((std::abs(max_y - min_y) % 2) == 1) { ++max_y; }
    // if((std::abs(max_z - min_z) % 2) == 1) { ++max_z; }

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
