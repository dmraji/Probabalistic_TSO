// Occupied voxel data, bound to each occupied voxel in hash table

#ifndef occ_data_hh
#define occ_data_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct occ_data
{
  // Occupancy probability
  float probability;

  // Size of side (for AMR)
  float sr_extent;

  // Intensity from LiDAR scan
  int intensity;

  occ_data() : probability(0.0f),
               sr_extent(1),
               intensity(0)
               {}
};

#endif
