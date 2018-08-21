// Occupied voxel data, bound to each occupied voxel in hash table

#ifndef occ_data_hh
#define occ_data_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct occ_data
{
  int hits;

  // Occupancy probability
  float probability;

  // Masked on or off based on probability
  bool mask;

  // Intensity from LiDAR scan
  float intensity;

  // Size of side (for AMR)
  int extent;

  occ_data() : hits(0),
               probability(0.0f),
               mask(false),
               intensity(0.0f),
               extent(1)
               {}
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct opp_data
{
  int hits;

  // Intensity from LiDAR scan
  float intensity;

  opp_data() : hits(0),
               intensity(0.0f)
               {}
};


#endif
