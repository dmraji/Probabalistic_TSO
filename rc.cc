#include <iostream>
#include <fstream>
#include <cstdio>

#include <string>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

// Boost Libraries
#include <boost/functional/hash.hpp>
// #include <boost/multi_index_container.hpp>
// #include <boost/multi_index/sequenced_index.hpp>
// #include <boost/multi_index/ordered_index.hpp>
// #include <boost/multi_index/hashed_index.hpp>
// #include <boost/multi_index/composite_key.hpp>
// #include <boost/multi_index/identity.hpp>
// #include <boost/multi_index/member.hpp>

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"

#include <sparsepp/spp.h>

// Data structure headers
#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

// Non-member function headers
#include "il_math.hh"
#include "vox_to_cents.hh"
#include "ray_cast.hh"
#include "occ_update.hh"
#include "bbx.hh"
#include "h5_read.hh"

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function
void timestamp(double start,
               std::string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  // Start clock
  std::clock_t start;
  start = std::clock();
  timestamp(start, "start");

  // Read data from H5 files
  h5_read reader;

  int cld_len = reader.sizeup("/cld");
  cldpt *cloud_scans = new cldpt[cld_len];
  reader.data_read_cld(cloud_scans,
                       cld_len
                       );

  int poses = reader.sizeup("/posData");
  pose *pose_pts = new pose[poses];
  rot_mat *rots = new rot_mat[poses];
  reader.data_read_pose(pose_pts,
                        rots,
                        poses
                        );

  // CONV TO ARGV
  int max_depth = 3;
  std::vector<int> depth_levels;

  for(int i = 0; i < max_depth; ++i)
  {
    depth_levels.push_back(std::pow(2, i));
  }

  // // Hash tables for occupied voxels
  // std::unordered_map <ind, opp_data> occ_per_pose;
  // std::unordered_map <ind, occ_data> box_occ;
  //
  // // Hash tables for free and unknown voxels
  // std::unordered_map <ind, free_unk_data> box_free;
  // std::unordered_map <ind, free_unk_data> box_unknown;

  // Hash tables for occupied voxels
  spp::sparse_hash_map <ind, opp_data> occ_per_pose;
  spp::sparse_hash_map <ind, occ_data> box_occ;

  // Hash tables for free and unknown voxels
  spp::sparse_hash_map <ind, free_unk_data> box_free;
  spp::sparse_hash_map <ind, free_unk_data> box_unknown;

  // Default resolution 10 cm
  float resolution = 0.1;

  // int cloud_cut = 0;
  // int cloud_chunk_len = int(floor(ptcld_len / path_len));

  // int scan_cld_cutoff = 0;
  int scan_pts = 0;

  for(int pose_ind = 0; pose_ind < 100; ++pose_ind)
  {
    int current_index = cloud_scans[scan_pts].scan_index;
    if(current_index == 0)
    {
      break;
    }
    std::cout << current_index << "vs" << pose_ind << '\n';
    // std::cout << scan_cld_cutoff << '\n';

    pt origin = { pose_pts[pose_ind].x,
                  pose_pts[pose_ind].y,
                  pose_pts[pose_ind].z };

    std::vector<pt> scan;
    scan.reserve(25000);
    // int scan_pts = scan_cld_cutoff;

    while(cloud_scans[scan_pts].scan_index == current_index)
    {
      scan.push_back( {cloud_scans[scan_pts].x,
                       cloud_scans[scan_pts].y,
                       cloud_scans[scan_pts].z,
                       cloud_scans[scan_pts].intensity} );
      ++scan_pts;
      if(scan_pts == cld_len)
      {
        break;
      }
    }
    // scan_cld_cutoff = scan_pts;

    for(int scan_ind = 0; scan_ind < scan.size(); ++scan_ind)
    {

      // pt end = scan[scan_ind];

      std::vector<pt> ray;
      cast_ray(origin,
               scan[scan_ind],
               resolution,
               ray
               );

      for(int i = 0; i < ray.size(); ++i)
      {
        // Mark free space
        ++box_free[ { f_floor(ray[i].x * (1/resolution)),
                      f_floor(ray[i].y * (1/resolution)),
                      f_floor(ray[i].z * (1/resolution)) } ].hits;
      }

      // Mark occupied space in temporary map
      ++occ_per_pose[ { f_floor(scan[scan_ind].x * (1/resolution)),
                        f_floor(scan[scan_ind].y * (1/resolution)),
                        f_floor(scan[scan_ind].z * (1/resolution)) } ].hits;

      occ_per_pose[ { f_floor(scan[scan_ind].x * (1/resolution)),
                      f_floor(scan[scan_ind].y * (1/resolution)),
                      f_floor(scan[scan_ind].z * (1/resolution)) } ].intensity = scan[scan_ind].intensity;
      // Ensure that destructor is called on ray vector
      std::vector<pt>().swap(ray);
    }

    vox_update(occ_per_pose,
               box_occ,
               box_free,
               box_unknown,
               pose_ind
               );

    timestamp(start,
              std::to_string(pose_ind));

    std::vector<pt>().swap(scan);

    std::cout << "occ_size: " << box_occ.size() << '\n';
    std::cout << "free_size: " << box_free.size() << '\n';
    std::cout << "unk_size: " << box_unknown.size() << '\n';
  }

  out_cents writer(box_occ,
                   box_free,
                   box_unknown,
                   resolution
                   );

  timestamp(start,
            "end"
            );

  return 0;
}
