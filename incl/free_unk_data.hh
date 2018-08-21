// Free and unknown voxel data, bound to each free or unknown voxel in hash table

#ifndef free_unk_data_hh
#define free_unk_data_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct free_unk_data
{
  int hits;

  free_unk_data() : hits(0)
                    {}
};

#endif
