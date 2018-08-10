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

// OcTree libs
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

std::atomic<bool> quit(false);    // signal flag

void got_signal(int)
{
  quit.store(true);
}

class sigint_check
{
  public:
    ~sigint_check() { std::cout << "destructor\n"; }
};

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

void print_query_info(point3d query, OcTreeNode* node)
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

double timestamp(double start,
                 string checkpt
                 )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << endl;
  return elapsed;
}

int main(int argc, char** argv)
{
  // Setup to catch sigint
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);

  sigint_check SI_C;
  int check = 0;
  while(check == 0)
  {
    std::clock_t start;
    start = std::clock();
    double startstamp = timestamp(start, "start");

    report_manager rep_man;

    // Read data from H5 files
    h5_read reader("RunData.h5");

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

    // Baseline usage after file read
    rep_man.file << timestamp(start, "data read") << ", " << getValue() << "\n";

    // Default octree constructor; create empty tree and set res of leafs to resolution
    float res = 0.1;
    OcTree tree(res);

    // std::vector<pt> occ;
    // std::vector<pt> unk;

    spp::sparse_hash_map<pt, occ_data> occ;
    spp::sparse_hash_map<pt, int> unk;

    point3d origin (0., 0., 0.);

    // std::cout << "Adding pointcloud to tree ... " << '\t';

    int scan_pts = 0;

    for(int pose_ind = 0; pose_ind < poses; ++pose_ind)
    {

      int current_index = cloud_scans[scan_pts].scan_index;
      if(current_index == 0)
      {
        break;
      }
      // std::cout << current_index << "vs" << pose_ind << '\n';
      // std::cout << scan_cld_cutoff << '\n';

      // int scan_pts = scan_cld_cutoff;

      octomap::Pointcloud cloudscan;

      while(cloud_scans[scan_pts].scan_index == current_index)
      {
        cloudscan.push_back(cloud_scans[scan_pts].x,
                            cloud_scans[scan_pts].y,
                            cloud_scans[scan_pts].z);
        ++scan_pts;
      }


      origin = point3d(pose_pts[pose_ind].x,
                       pose_pts[pose_ind].y,
                       pose_pts[pose_ind].z);

      tree.insertPointCloud(cloudscan,
                            origin,
                            -1.,
                            false,
                            false
                            );

      // point3d l_bnd (0., 0., 0.);
      // point3d u_bnd (0., 0., 0.);

      // cloudscan.calcBBX(l_bnd, u_bnd);

      point3d query (0., 0., 0.);
      OcTreeNode* result = tree.search(query);

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

      pt l_bnd = {0., 0., 0.};
      pt u_bnd = {0., 0., 0.};
      for(OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
      {
      //   //manipulate node, e.g.:
      //   // std::cout << "Node center: " << it.getCoordinate() << std::endl;
      //   // if(it.getSize() > 0.1f)
      //   // {
      //   //   std::cout << "Node size: " << it.getSize() << std::endl;
      //   //   std::cout << "Node value: " << it->getValue() << std::endl;
      //   // }
        // std::cout << "Node value: " << it->getValue() << std::endl;
        if(tree.isNodeOccupied(*it))
        {
          occ[ {it.getCoordinate().x(),
                it.getCoordinate().y(),
                it.getCoordinate().z()} ].probability = it->getOccupancy();
          occ[ {it.getCoordinate().x(),
                it.getCoordinate().y(),
                it.getCoordinate().z()} ].sr_extent = it.getSize();

          if(it->getOccupancy() > 0.9)
          {
            if(it.getCoordinate().x() < l_bnd.x) { l_bnd.x = it.getCoordinate().x(); }
            else { if(it.getCoordinate().x() > u_bnd.x) { u_bnd.x = it.getCoordinate().x(); } }
            if(it.getCoordinate().y() < l_bnd.y) { l_bnd.y = it.getCoordinate().y(); }
            else { if(it.getCoordinate().y() > u_bnd.y) { u_bnd.y = it.getCoordinate().y(); } }
            if(it.getCoordinate().z() < l_bnd.z) { l_bnd.z = it.getCoordinate().z(); }
            else { if(it.getCoordinate().z() > u_bnd.z) { u_bnd.z = it.getCoordinate().z(); } }
          }
        }
      }
      std::cout << l_bnd.x << " " << u_bnd.x << '\n';
      std::cout << l_bnd.y << " " << u_bnd.y << '\n';
      std::cout << l_bnd.z << " " << u_bnd.z << '\n';

      for(double ix = l_bnd.x; ix < u_bnd.x; ix += res)
      {
        for(double iy = l_bnd.y; iy < u_bnd.y; iy += res)
        {
          for(double iz = l_bnd.z; iz < u_bnd.z; iz += res)
          {
            if(!tree.search(ix, iy, iz))
            {
              // std::cout << "unknown" << '\n';
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

      cloudscan.clear();
      double posestamp = timestamp(start, std::to_string(pose_ind));
      int mem_in_use = getValue();
      // std::cout << mem_in_use << '\n';
      rep_man.file << posestamp << ", " << mem_in_use << "\n";

      // Check for sigint;
      if(quit.load()) goto BREAKER;

    }

    out_cents writer(occ,
                     unk
                     );

    // origin = point3d(12., 12.5, -11.);
    //
    // octomap::KeySet free_cells, occupied_cells;
    //
    // tree.computeDiscreteUpdate(cloudscan,
    //                            origin,
    //                            free_cells,
    //                            occupied_cells,
    //                            -1.);

    // OcTreeNode: represents 3D occupancy grid cell;
    //   In this case, "result" stores the cell's log-odds occupancy

    // Octree.search: searches tree node at specified depth given 3D point;
    //   Important to check whether returned node is NULL (occurs if it is in unkown space)
    // print_query_info(query, result);
    //
    // query = point3d(1.,1.,1.);
    // result = tree.search (query);
    // print_query_info(query, result);
    //
    // point3d origin (0., 0., 0.);
    // point3d dir (0., 0., 0.);
    // point3d end (0., 0., 0.);
    // origin = point3d(0., 0., 0.);
    // dir = point3d(1., 1., 1.);
    // end = point3d(2., 2., 2.);
    // bool ray_0 = tree.castRay(origin, dir, end, false, 0);
    //
    // std::cout << ray_0 << '\n';
    //
    // key_0 = coordToKey()
    // std::vector<KeyRay> rays;
    // rays.push_back()
    // tree.computeRay(origin, end, rays)
    //
    // cout << endl;

    // OcTree.writeBinary: writes OcTree to binary file;
    //   Tree is first converted to MLE and then "pruned" before writing
    tree.writeBinary("octoccupancy_tree.bt");
    cout << endl;
    check = 1;
  }

  BREAKER:
    ;

  return 0;

  // cout << "now you can use octovis to visualize: octovis template_tree.bt"  << endl;
  // cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;
}
