// Update occupancy with new scan data (header)

#ifndef occ_update_hh
#define occ_update_hh

#include <iostream>

#include <unordered_map>
#include <utility>
#include <functional>

// Sparse hash table
#include <sparsepp/spp.h>

#include "ind.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"
#include "bbx.hh"

void vox_update(spp::sparse_hash_map<ind, free_unk_data> & opp,
                spp::sparse_hash_map<ind, occ_data> & occ,
                spp::sparse_hash_map<ind, free_unk_data> & freev,
                spp::sparse_hash_map<ind, free_unk_data> & unk,
                int pose_ind,
                int max_depth
                );

float prob_update(spp::sparse_hash_map<ind, occ_data> & occ
                 );

void parse_bbx(spp::sparse_hash_map<ind, occ_data> & occ,
               spp::sparse_hash_map<ind, free_unk_data> & freev,
               spp::sparse_hash_map<ind, free_unk_data> & unk,
               corners &bounds,
               corners &prior_bounds,
               int depthl,
               int max_depth
               );

struct adjust_report
{
  bool exist;
  int inc;
};

adjust_report adj_extent(spp::sparse_hash_map<ind, free_unk_data> & vox,
                         ind cind,
                         int depthl
                         );

adjust_report adj_extent(spp::sparse_hash_map<ind, occ_data> & vox,
                         ind cind,
                         int depthl
                         );

// void prune(spp::sparse_hash_map<ind, occ_data> & occ,
//            spp::sparse_hash_map<ind, free_unk_data> & freev,
//            spp::sparse_hash_map<ind, free_unk_data> & unk
//            );

#endif
