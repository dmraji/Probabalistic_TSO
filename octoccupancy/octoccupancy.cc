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

  cout << endl;
  std::cout << "Init main." << '\n';

  // Default octree constructor; create empty tree and set res of leafs to double
  float res = 0.1;
  OcTree tree(res);

  point3d origin (0., 0., 0.);

  int pos_len = 525;

  float** pos = new float*[pos_len];
  for(int i = 0; i < pos_len; i++)
  {
    pos[i] = new float[3];
  }

  std::cout << "Parsing pos from file ... \t";

  fstream p_in("pos.txt");
  string p_line;

  int i = 0, j = 0;
  while(getline(p_in, p_line))
  {

    float p_value;

    j = 0;
    stringstream p_ss(p_line);

    while(p_ss >> p_value)
    {
      pos[i][j] = p_value;
      ++j;
    }
    ++i;
  }

  std::cout << "Done." << endl;

  int ptcld_len = 14563019;
  int sub_len = int(floor(ptcld_len / 525));

  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0));

  std::cout << "Parsing pointcloud from file ... \t";

  fstream cl_in("ptcld.txt");
  string cl_line;

  i = 0, j = 0;
  // int cut = 0;
  // int cutoff = sub_len;
  while(getline(cl_in, cl_line))
  {
    // if(cut == cutoff)
    // {
    //   cutoff = cutoff + sub_len;
    //   break;
    // }
    float cl_value;

    j = 0;
    stringstream cl_ss(cl_line);

    while(cl_ss >> cl_value)
    {
      ptcld[i][j] = cl_value;
      ++j;
    }
    ++i;
    // ++cut;
  }

  std::cout << "Done." << endl;

  // std::cout << "Broadcasting pointcloud to octomap object ... " << '\t';
  //
  // point3d l_bnd (0., 0., 0.);
  // point3d u_bnd (0., 0., 0.);
  //
  // cloudscan.calcBBX(l_bnd, u_bnd);

  std::cout << "Done." << endl;

  timestamp(start, "Pre-add pointcloud to tree");

  std::cout << "Adding pointcloud to tree ... " << '\t';

  int cloud_ind = 0;
  for(int i = 0; i < pos_len; ++i)
  {
    octomap::Pointcloud cloudscan;

    for(int j = cloud_ind; j < cloud_ind + sub_len; ++j)
    {
      // std::cout << j << '\n';
      cloudscan.push_back(ptcld[j][0], ptcld[j][1], ptcld[j][2]);
    }
    // std::cout << "146" << '\n';
    cloud_ind = cloud_ind + sub_len;

    origin = point3d(pos[i][0], pos[i][1], pos[i][2]);

    tree.insertPointCloud(cloudscan,
                          origin,
                          -1.,
                          false,
                          false
                          );

    cloudscan.clear();
    timestamp(start, std::to_string(i));
  }

  // origin = point3d(12., 12.5, -11.);
  //
  // octomap::KeySet free_cells, occupied_cells;
  //
  // tree.computeDiscreteUpdate(cloudscan,
  //                            origin,
  //                            free_cells,
  //                            occupied_cells,
  //                            -1.);

  std::cout << "Done." << endl;

  // point3d query (0., 0., 0.);
  // OcTreeNode* result = tree.search(query);

  // for (double ix = l_bnd.x(); ix < u_bnd.x(); ix += res)
  // {
  //   for (double iy = l_bnd.y(); iy < u_bnd.y(); iy += res)
  //   {
  //     for (double iz = l_bnd.z(); iz < u_bnd.z(); iz += res)
  //     {
  //       if (!tree.search(ix, iy, iz))
  //       {
  //         std::cout << "unknown" << '\n';
  //       }
  //       else
  //       {
  //         query = point3d(ix, iy, iz);
  //         result = tree.search(query);
  //         print_query_info(query, result);
  //       }
  //     }
  //   }
  // }

  // OcTreeNode: represents 3D occupancy grid cell;
  //   In this case, "result" stores the cell's log-odds occupancy

  // Octree.search: searches tree node at specified depth given 3D point;
  //   Important to check whether returned node is NULL (occurs if it is in unkown space)
  // print_query_info(query, result);
  //

  //
  // query = point3d(1.,1.,1.);
  // result = tree.search (query);
  // print_query_info(query, result);
  //
  // for(OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
  // {
  //   //manipulate node, e.g.:
  //   std::cout << "Node center: " << it.getCoordinate() << std::endl;
  //   std::cout << "Node size: " << it.getSize() << std::endl;
  //   std::cout << "Node value: " << it->getValue() << std::endl;
  // }
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

  // key_0 = coordToKey()
  // std::vector<KeyRay> rays;
  // rays.push_back()
  // tree.computeRay(origin, end, rays)

  cout << endl;

  // OcTree.writeBinary: writes OcTree to binary file;
  //   Tree is first converted to MLE and then "pruned" before writing
  tree.writeBinary("octoccupancy_tree.bt");
  cout << endl;

  // cout << "now you can use octovis to visualize: octovis template_tree.bt"  << endl;
  // cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;
}
