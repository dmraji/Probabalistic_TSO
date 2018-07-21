#ifndef occ_data_hh
#define occ_data_hh

// Occupied voxel data
struct occ_data
{
  int hits;
  int sr_extent;
  float probability;
  bool mask;
};

#endif
