// Occupied voxel data, bound to each occupied voxel in hash table

#ifndef occ_data_hh
#define occ_data_hh

struct occ_data
{
  float probability;
  float sr_extent;
  int intensity;

  occ_data() : probability(0.0f),
               sr_extent(1),
               intensity(0)
               {}
};

#endif
