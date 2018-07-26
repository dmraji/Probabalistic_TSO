#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <vector>
#include <iterator>
#include <ctime>

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

void timestamp(double start,
               string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << endl;
}

int main(int argc, char** argv)
{

  std::clock_t start;
  start = std::clock();
  timestamp(start, "start");

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

  // Default octree constructor; create empty tree and set res of leafs to resolution
  float res = 0.1;
  OcTree tree(res);

  std::vector<pt> occ;
  std::vector<pt> unk;

  point3d origin (0., 0., 0.);

  // std::cout << "Adding pointcloud to tree ... " << '\t';

  int scan_pts = 0;

  for(int pose_ind = 0; pose_ind < poses; ++pose_ind)
  {

    int current_index = cloud_scans[scan_pts].scan_index;
    // std::cout << current_index << "vs" << pose_ind << '\n';
    // std::cout << scan_cld_cutoff << '\n';

    if(current_index != pose_ind)
    {
      continue;
    }

    std::vector<pt> scan;
    scan.reserve(25000);
    // int scan_pts = scan_cld_cutoff;

    while(cloud_scans[scan_pts].scan_index == current_index)
    {
      scan.push_back( {cloud_scans[scan_pts].x,
                       cloud_scans[scan_pts].y,
                       cloud_scans[scan_pts].z} );
      ++scan_pts;
    }

    octomap::Pointcloud cloudscan;

    for(int j = 0; j < scan.size(); ++j)
    {
      // std::cout << j << '\n';
      cloudscan.push_back(scan[j].x, scan[j].y, scan[j].z);
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

    point3d l_bnd (0., 0., 0.);
    point3d u_bnd (0., 0., 0.);

    cloudscan.calcBBX(l_bnd, u_bnd);

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
    //         unk.push_back( {ix, iy, iz} );
    //       }
    //       else
    //       {
    //         query = point3d(ix, iy, iz);
    //         result = tree.search(query);
    //         print_query_info(query, result);
    //         if(result != 0)
    //         {
    //           occ.push_back( {ix, iy, iz} );
    //         }
    //       }
    //     }
    //   }
    // }

    // IMPLEMENTATION #2 - leaf_iterator comb for occupied voxels, black box "getUnknownLeafCenters" member function for unknown centers

    for(OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
    {
      //manipulate node, e.g.:
      // std::cout << "Node center: " << it.getCoordinate() << std::endl;
      // std::cout << "Node size: " << it.getSize() << std::endl;
      // std::cout << "Node value: " << it->getValue() << std::endl;
      occ.push_back( {it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()} );
    }

    octomap::point3d_list unknowns_leaf_list;
    tree.getUnknownLeafCenters(unknowns_leaf_list,
                                   l_bnd,
                                   u_bnd
                                   );

    std::vector<point3d> unk_p3d{ std::begin(unknowns_leaf_list), std::end(unknowns_leaf_list) };
    unk.reserve(unk_p3d.size());
    for(int i = 0; i < unk_p3d.size(); ++i)
    {
      unk.push_back( {unk_p3d[i].x(), unk_p3d[i].y(), unk_p3d[i].z()} );
    }

    std::vector<pt>().swap(scan);
    cloudscan.clear();
    timestamp(start, std::to_string(current_index));

    // RESULT #1: Threw "std::bad_alloc" after ~12 minutes and roughly 200 pose-scans

    // RESULT #2: Stall after ~100 pose-scans/5 minutes; syscheck revealed 13gigs of memory usage; manually killed;
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

  // cout << "now you can use octovis to visualize: octovis template_tree.bt"  << endl;
  // cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;
}
