#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

#include <vector>
#include <iterator>
#include <ctime>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include <sparsepp/spp.h>

// Octomap libs
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"

#include "h5_read.hh"
#include "vox_to_cents.hh"
#include "ind.hh"
#include "pt.hh"

using namespace std;

// Octomap namespace
using namespace octomap;

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

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

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

int getValue()
{ //Note: this value is in KB!
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

// Coutput information about node in tree
void print_query_info(point3d query,
                      OcTreeNode* node
                      )
{
  if (node != NULL)
  {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
  {
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function
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

int main(int argc, char** argv)
{
  // Setup to catch sigint and terminate properly
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);

  sigint_check SI_C;
  int check = 0;

  // Loop exits upon sigint
  while(check == 0)
  {
    // Start clock
    std::clock_t start;
    start = std::clock();
    double startstamp = timestamp(start, "start");

    // Instantiate report manager
    report_manager rep_man;

    // Instantiate H5 Reader
    h5_read reader("RunData.h5"
                   );

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

    // Default octree constructor; create empty tree and set res of leafs to resolution
    float res = 0.1;
    OcTree tree(res);

    // Sparse hash tables to handle voxel centers
    spp::sparse_hash_map<pt, occ_data> occ;
    spp::sparse_hash_map<pt, int> unk;

    // Define initial pose
    point3d origin (0., 0., 0.);

    int scan_pts = 0;

    // Iterate through pose
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

      // int scan_pts = scan_cld_cutoff;

      // Instantiate cloudscan object
      octomap::Pointcloud cloudscan;

      // Iterate through points in cloud for current index
      while(cloud_scans[scan_pts].scan_index == current_index)
      {
        // Add proper points from scan to cloudscan object
        cloudscan.push_back(cloud_scans[scan_pts].x,
                            cloud_scans[scan_pts].y,
                            cloud_scans[scan_pts].z);
        ++scan_pts;
      }

      // Origin set to current pose for raycasting
      origin = point3d(pose_pts[pose_ind].x,
                       pose_pts[pose_ind].y,
                       pose_pts[pose_ind].z);

      // Add pointcloud object to the tree
      tree.insertPointCloud(cloudscan,
                            origin,
                            -1.,
                            false,
                            false
                            );

      // point3d l_bnd (0., 0., 0.);
      // point3d u_bnd (0., 0., 0.);

      // cloudscan.calcBBX(l_bnd, u_bnd);

      // Define initial query
      point3d query (0., 0., 0.);

      // Define initial result of query
      OcTreeNode* result = tree.search(query);

      // Reset unknown map
      unk.clear();

      // IMPLEMENTATION #1 - manual comb of bbx for unknown and occupied leafs

      // for (double ix = l_bnd.x(); ix < u_bnd.x(); ix += res)
      // {
      //   for (double iy = l_bnd.y(); iy < u_bnd.y(); iy += res)
      //   {
      //     for (double iz = l_bnd.z(); iz < u_bnd.z(); iz += res)
      //     {
      //       if (!tree.search(ix, iy, iz))
      //       {
      //         // std::cout << "unknown" << '\n';
      //         ++unk[ {ix, iy, iz} ];
      //       }
      //     }
      //   }
      // }

      // IMPLEMENTATION #2 - leaf_iterator comb for occupied voxels, black box "getUnknownLeafCenters" member function for unknown centers

      // Initial corners for bounding box
      pt l_bnd = {0., 0., 0.};
      pt u_bnd = {0., 0., 0.};

      // Iterate through leaves of tree
      for(OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
      {
        // Query whether node is occupied according to current threshold
        if(tree.isNodeOccupied(*it))
        {
          // Get the occupancy of the node
          occ[ {it.getCoordinate().x(),
                it.getCoordinate().y(),
                it.getCoordinate().z()} ].probability = it->getOccupancy();

          // Get the size of the node
          occ[ {it.getCoordinate().x(),
                it.getCoordinate().y(),
                it.getCoordinate().z()} ].sr_extent = it.getSize();

          // Arbitrary threshold used to determine bounding box corners
          if(it->getOccupancy() > 0.9)
          {
            // If node above threshold is farther than current corner, update corner value
            if(it.getCoordinate().x() < l_bnd.x) { l_bnd.x = it.getCoordinate().x(); }
            else { if(it.getCoordinate().x() > u_bnd.x) { u_bnd.x = it.getCoordinate().x(); } }
            if(it.getCoordinate().y() < l_bnd.y) { l_bnd.y = it.getCoordinate().y(); }
            else { if(it.getCoordinate().y() > u_bnd.y) { u_bnd.y = it.getCoordinate().y(); } }
            if(it.getCoordinate().z() < l_bnd.z) { l_bnd.z = it.getCoordinate().z(); }
            else { if(it.getCoordinate().z() > u_bnd.z) { u_bnd.z = it.getCoordinate().z(); } }
          }
        }
      }

      // Coutput corners of bounding box
      std::cout << l_bnd.x << " " << u_bnd.x << '\n';
      std::cout << l_bnd.y << " " << u_bnd.y << '\n';
      std::cout << l_bnd.z << " " << u_bnd.z << '\n';

      // Iterate through bounding box by resolution
      for(double ix = l_bnd.x; ix < u_bnd.x; ix += res)
      {
        for(double iy = l_bnd.y; iy < u_bnd.y; iy += res)
        {
          for(double iz = l_bnd.z; iz < u_bnd.z; iz += res)
          {
            // If node is not in tree, add it to unknown map
            if(!tree.search(ix, iy, iz))
            {
              ++unk[ {ix, iy, iz} ];
            }
          }
        }
      }

      // octomap::point3d_list unknowns_leaf_list;
      // tree.getUnknownLeafCenters(unknowns_leaf_list,
      //                            l_bnd,
      //                            u_bnd
      //                            );
      // //
      // std::vector<point3d> unk_p3d{ std::begin(unknowns_leaf_list), std::end(unknowns_leaf_list) };
      // for(int i = 0; i < unk_p3d.size(); ++i)
      // {
      //   ++unk[ {unk_p3d[i].x(),
      //           unk_p3d[i].y(),
      //           unk_p3d[i].z()} ];
      // }

      // Reset cloudscan object
      cloudscan.clear();

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
    out_cents writer(occ,
                     unk
                     );

    // OcTree.writeBinary: writes OcTree to binary file;
    // Tree is first converted to MLE and then "pruned" before writing
    tree.writeBinary("octoccupancy_tree.bt");
    cout << endl;

    // Prevent endless looping while checking for sigint; terminate normally if no sigint
    check = 1;
  }

  // Release valve for sigint
  BREAKER:
    ;

  return 0;

  // cout << "now you can use octovis to visualize: octovis template_tree.bt"  << endl;
}
