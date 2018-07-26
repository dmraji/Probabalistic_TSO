// Free and unknown voxel data, bound to each free or unknown voxel in hash table

#ifndef free_unk_data_hh
#define free_unk_data_hh

struct free_unk_data
{
  int hits;
  int sr_extent;

  free_unk_data() : hits(0),
                    sr_extent(1)
                    {}
};

#endif
