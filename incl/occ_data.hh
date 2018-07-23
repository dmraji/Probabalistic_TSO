// Occupied voxel data, bound to each occupied voxel in hash table

#ifndef occ_data_hh
#define occ_data_hh

struct occ_data
{
  int hits;
  int sr_extent;
  float probability;
  bool mask;

  occ_data() : hits(0),
               sr_extent(1),
               probability(0.0f),
               mask(false)
               {}
};

#endif
