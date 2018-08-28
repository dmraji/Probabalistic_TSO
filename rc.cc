#include <iostream>
#include <fstream>
#include <cstdio>

#include <string>

#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

#include "stdlib.h"
#include "stdio.h"

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"

// For Sparse Hash Map Implementation; See https://github.com/greg7mdp/sparsepp
#include <sparsepp/spp.h>

// Data structure headers
#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

// Non-member function and class headers
#include "il_math.hh"
#include "vox_to_cents.hh"
#include "ray_cast.hh"
#include "occ_update.hh"
#include "bbx.hh"
#include "h5_read.hh"

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// TO ENSURE USAGE DATA STILL WRITTEN IN CASE OF SIGINT (BEGIN)
std::atomic<bool> quit(false);

void got_signal(int)
{
  quit.store(true);
}

class sigint_check
{
  public:
    ~sigint_check() { std::cout << "destructor\n"; }
};
// TO ENSURE USAGE DATA STILL WRITTEN IN CASE OF SIGINT (END)

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// USAGE REPORT (MEMORY) DATA RETRIEVAL (BEGIN)
struct report_manager
{
  std::ofstream file;

  report_manager()
  {
    // Start report file
    file.open("usage_rep.txt");
  }
  ~report_manager()
  {
    file.close();
  }
};

int parseLine(char* line)
{
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char* p = line;
  while (*p <'0' || *p > '9') p++;
  line[i-3] = '\0';
  i = atoi(p);
  return i;
}

// RETURN VALUE WILL BE IN KiB!
int getValue()
{
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL)
  {
    if (strncmp(line, "VmRSS:", 6) == 0)
    {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}
// USAGE REPORT (MEMORY) DATA RETRIEVAL (END)

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function for usage report and debugging
double timestamp(double start,
                 string checkpt
                 )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << endl;
  return elapsed;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  // Setup to catch sigint and terminate properly
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);

  sigint_check SI_C;
  int check = 1;

  // Loop exits upon sigint
  while(check == 1)
  {

  // Start clock
  std::clock_t start;
  start = std::clock();
  double startstamp = timestamp(start, "start");

  // Instantiate report manager
  report_manager rep_man;

  // Instantiate H5 Reader
  h5_read reader;

  // Parse and read pointcloud data
  int cld_len = reader.sizeup("/cld");
  cldpt *cloud_scans = new cldpt[cld_len];
  reader.data_read_cld(cloud_scans,
                       cld_len
                       );

  // Parse and read pose data
  int poses = reader.sizeup("/posData");
  pose *pose_pts = new pose[poses];
  rot_mat *rots = new rot_mat[poses];
  reader.data_read_pose(pose_pts,
                        rots,
                        poses
                        );

  // Baseline usage after file read; Subtract this from all other usage values to get algorithm usage
  rep_man.file << timestamp(start, "data read") << ", " << getValue() << "\n";

  // Hash tables for occupied voxels
  spp::sparse_hash_map <ind, opp_data> occ_per_pose;
  spp::sparse_hash_map <ind, int> pocc;
  spp::sparse_hash_map <ind, occ_data> box_occ;

  // Hash tables for free and unknown voxels
  spp::sparse_hash_map <ind, free_unk_data> box_free;
  spp::sparse_hash_map <ind, free_unk_data> box_unknown;

  // Default resolution 10 cm
  float resolution = 0.1;

  // int scan_cld_cutoff = 0;
  int scan_pts = 0;

  // Iterate through poses
  for(int pose_ind = 0; pose_ind < poses; ++pose_ind)
  {
    // Current index set to scan index of current pointcloud
    int current_index = cloud_scans[scan_pts].scan_index;

    // Prevent issues with pointclouds that lack pose
    if(current_index == 0)
    {
      break;
    }
    // std::cout << current_index << "vs" << pose_ind << '\n';
    // std::cout << scan_cld_cutoff << '\n';

    // Origin set to current pose for raycasting
    pt origin = { pose_pts[pose_ind].x,
                  pose_pts[pose_ind].y,
                  pose_pts[pose_ind].z };

    // Reserve space for all points in scan
    std::vector<pt> scan;
    scan.reserve(25000);
    // int scan_pts = scan_cld_cutoff;

    // Dump all current index points from cloud into scan vector
    while(cloud_scans[scan_pts].scan_index == current_index)
    {
      scan.push_back( {cloud_scans[scan_pts].x,
                       cloud_scans[scan_pts].y,
                       cloud_scans[scan_pts].z,
                       cloud_scans[scan_pts].intensity} );
      ++scan_pts;
      // if(scan_pts == cld_len)
      // {
      //   break;
      // }
    }
    // scan_cld_cutoff = scan_pts;

    // Iterate through scan points for current index
    for(int scan_ind = 0; scan_ind < scan.size(); ++scan_ind)
    {

      std::vector<pt> ray;

      // Call to raycasting non-member function
      cast_ray(origin,
               scan[scan_ind],
               resolution,
               ray
               );

      // Iterate through casted ray
      for(int i = 0; i < ray.size(); ++i)
      {
        // Mark free space
        ind cind = { f_floor(ray[i].x * (1/resolution)),
                     f_floor(ray[i].y * (1/resolution)),
                     f_floor(ray[i].z * (1/resolution)) };
        ++box_free[cind].hits;

        // If free space was previously occupied, decrement occupancy
        if(box_occ.count(cind) != 0)
        {
          if(box_occ[cind].hits > 0) { --box_occ[cind].hits; }
        }
      }

      // Mark occupied space in temporary map to avoid bunch-biasing
      ++occ_per_pose[ { f_floor(scan[scan_ind].x * (1/resolution)),
                        f_floor(scan[scan_ind].y * (1/resolution)),
                        f_floor(scan[scan_ind].z * (1/resolution)) } ].hits;

      // Average intensity values from LiDAR scan
      occ_per_pose[ { f_floor(scan[scan_ind].x * (1/resolution)),
                      f_floor(scan[scan_ind].y * (1/resolution)),
                      f_floor(scan[scan_ind].z * (1/resolution)) } ].intensity += scan[scan_ind].intensity;

      // Ensure that destructor is called on ray vector
      std::vector<pt>().swap(ray);
    }

    // Call to voxel updater
    vox_update(occ_per_pose,
               pocc,
               box_occ,
               box_free,
               box_unknown,
               pose_ind
               );

    // Ensure destructor is called on scan vector
    std::vector<pt>().swap(scan);

    // Coutput occupied, free and unknown voxels
    std::cout << "occ_size: " << box_occ.size() << '\n';
    std::cout << "free_size: " << box_free.size() << '\n';
    std::cout << "unk_size: " << box_unknown.size() << '\n';

    // Timestamp for usage report
    double posestamp = timestamp(start,
                                 "pose #"+std::to_string(pose_ind));
    // Memory for usage report
    int mem_in_use = getValue();
    // std::cout << mem_in_use << '\n';

    // Write usage data to file
    rep_man.file << posestamp << ", " << mem_in_use << "\n";

    // Check for sigint
    if(quit.load()) goto BREAKER;
  }

  // Instantiation of data writer (process done in construction)
  out_cents writer(box_occ,
                   box_free,
                   box_unknown,
                   pocc,
                   resolution
                   );

  // Ending timestamp
  double endstamp = timestamp(start,
                              "end"
                              );

  // Prevent endless looping while checking for sigint; terminate normally if no sigint
  check = 0;
  }

  // Release valve for sigint
  BREAKER:
    ;

  return 0;
}
