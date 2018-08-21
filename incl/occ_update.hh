// Update occupancy with new scan data (header)

#ifndef occ_update_hh
#define occ_update_hh

#include <unordered_map>
#include <utility>
#include <functional>

#include <sparsepp/spp.h>

#include "ind.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"
#include "bbx.hh"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void vox_update(spp::sparse_hash_map<ind, opp_data> & opp,
                spp::sparse_hash_map<ind, int> & pocc,
                spp::sparse_hash_map<ind, occ_data> & occ,
                spp::sparse_hash_map<ind, free_unk_data> & freev,
                spp::sparse_hash_map<ind, free_unk_data> & unk,
                int pose_ind
                );

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

float prob_update(spp::sparse_hash_map<ind, occ_data> & occ,
                  int pose_ind
                  );

#endif
