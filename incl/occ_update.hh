// Update occupancy with new scan data (header)

#ifndef occ_update_hh
#define occ_update_hh

#include <unordered_map>
#include <utility>
#include <functional>

#include "ind.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"
#include "bbx.hh"

void vox_update(std::unordered_map<ind, free_unk_data> & opp,
                std::unordered_map<ind, occ_data> & occ,
                std::unordered_map<ind, free_unk_data> & freev,
                std::unordered_map<ind, free_unk_data> & unk,
                int pose_ind,
                int max_depth,
                float max_thresh
                );

float prob_update(std::unordered_map<ind, occ_data> & occ
                 );

void parse_bbx(std::unordered_map<ind, occ_data> & occ,
               std::unordered_map<ind, free_unk_data> & freev,
               std::unordered_map<ind, free_unk_data> & unk,
               corners bounds
               );

#endif
