// Free and unknown voxel data, bound to each free or unknown voxel in hash table

#ifndef free_unk_data_hh
#define free_unk_data_hh

struct free_unk_data
{
  int hits;

  free_unk_data() : hits(0)
                    {}
};

struct opp_data
{
  int hits;
  float intensity;

  opp_data() : hits(0),
               intensity(0.0f)
               {}
};

#endif
