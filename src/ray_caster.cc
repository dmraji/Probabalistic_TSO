#include <iostream>
#include <ctime>

#include <cmath>
#include <vector>
#include <random>
#include <iterator>

#include "ray_caster.hh"
#include "boost/multi_array.hpp"
#include <cassert>

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

ray_caster::ray_caster(vector< vector<float> > & ptcld,
                       int ptcld_len,
                       vector< vector<float> > & path,
                       int path_len,
                       vector< vector<float> > & boundaries,
                       double start
                       )
{

  diff_x = abs(boundaries[0][0] - boundaries[1][0]);
  diff_y = abs(boundaries[0][1] - boundaries[1][1]);
  diff_z = abs(boundaries[0][2] - boundaries[1][2]);

  // std::cout << "diff_x: " << diff_x << '\n';
  // std::cout << "diff_y: " << diff_y << '\n';
  // std::cout << "diff_z: " << diff_z << '\n';

  min_x = boundaries[0][0];
  min_y = boundaries[0][1];
  min_z = boundaries[0][2];
  max_x = boundaries[1][0];
  max_y = boundaries[1][1];
  max_z = boundaries[1][2];

  // x_pts = 20;
  // y_pts = 20;
  // z_pts = 5;
  // tot_pts = x_pts*y_pts*z_pts;

  resolution = 0.5;
  spaced_x = g_range(min_x, resolution, max_x);
  x_pts = spaced_x.size();
  // x_neg = bifurcate(spaced_x, -1);
  // x_pos = bifurcate(spaced_x, 1);
  spaced_y = g_range(min_y, resolution, max_y);
  y_pts = spaced_y.size();
  // y_neg = bifurcate(spaced_y, -1);
  // y_pos = bifurcate(spaced_y, 1);
  spaced_z = g_range(min_z, resolution, max_z);
  z_pts = spaced_z.size();
  // z_neg = bifurcate(spaced_z, -1);
  // z_pos = bifurcate(spaced_z, 1);

  tot_pts = spaced_x.size() * spaced_y.size() * spaced_z.size();

  // std::vector< std::vector<float> > box(tot_pts, std::vector<float>(3, 0));

  // Organize box pts in chunks of z-space for easier parsing during raycasting
  // box_ind = 0;
  // for(int k = 0; k < z_pts; ++k)
  // {
  //   for(int i = 0; i < x_pts; ++i)
  //   {
  //     for(int j = 0; j < y_pts; ++j)
  //     {
  //       box[box_ind][0] = spaced_x[i];
  //       box[box_ind][1] = spaced_y[j];
  //       box[box_ind][2] = spaced_z[k];
  //
  //       ++box_ind;
  //     }
  //   }
  // }

  typedef boost::multi_array<float, 4> array_type;
  typedef array_type::index index;
  array_type box(boost::extents[x_pts][y_pts][z_pts][3]);

  for(index i = 0; i < x_pts; ++i)
  {
    for(index j = 0; j < y_pts; ++j)
    {
      for(index k = 0; k < z_pts; ++k)
      {
        box[i][j][k][0] = spaced_x[i];
        box[i][j][k][1] = spaced_y[j];
        box[i][j][k][2] = spaced_z[k];
      }
    }
  }

  typedef boost::multi_array<int, 3> array_type_3d;
  typedef array_type_3d::index index_3d;
  array_type_3d box_hit(boost::extents[x_pts][y_pts][z_pts]);
  array_type_3d box_miss(boost::extents[x_pts][y_pts][z_pts]);

  for(index_3d ii = 0; ii < x_pts; ++ii)
  {
    for(index_3d jj = 0; jj < y_pts; ++jj)
    {
      for(index_3d kk = 0; kk < z_pts; ++kk)
      {
        box_hit[ii][jj][kk] = 0;
        box_miss[ii][jj][kk] = 0;
      }
    }
  }

  ray_caster::timestamp(start,
                        "pre_path"
                        );

  // typedef boost::multi_array<double, 3> array_type;
  // typedef array_type::index index;
  // array_type A(boost::extents[3][4][2]);
  //
  // // Assign values to the elements
  // int values = 0;
  // for(index i = 0; i != 3; ++i)
  //   for(index j = 0; j != 4; ++j)
  //     for(index k = 0; k != 2; ++k)
  //       A[i][j][k] = values++;
  //
  // // Verify values
  // int verify = 0;
  // for(index i = 0; i != 3; ++i)
  //   for(index j = 0; j != 4; ++j)
  //     for(index k = 0; k != 2; ++k)
  //       assert(A[i][j][k] == verify++);

  // Init vars for path iteration
  // std::vector<float> dist(3);
  // std::vector<float> dir(3);

  std::vector<int> origin(3);
  std::vector<int> point_in_cloud(3);
  std::vector<int> direc(3);
  std::vector<int> pt(3);
  std::vector<int> inds(3);
  std::vector<float> probs(3);

  cloud_cut = 0;
  cloud_chunk_len = int(floor(ptcld_len / path_len));
  for(int o = 0; o < path_len; ++o)
  {

    // Position origin at index within bounding box
    m_or_prox = 100;
    for(index i = 0; i < x_pts; ++i)
    {
      for(index j = 0; j < y_pts; ++j)
      {
        for(index k = 0; k < z_pts; ++k)
        {

          prox_2 = (box[i][j][k][0] - path[o][0])*(box[i][j][k][0] - path[o][0]) + (box[i][j][k][1] - path[o][1])*(box[i][j][k][1] - path[o][1]) + (box[i][j][k][2] - path[o][2])*(box[i][j][k][2] - path[o][2]);

          if(prox_2 < m_or_prox)
          {
            m_or_prox = prox_2;
            origin[0] = int(i);
            origin[1] = int(j);
            origin[2] = int(k);
          }
        }
      }
    }

    // Build chunk of ptcld
    std::vector< std::vector<float> > ptcld_chunk(cloud_chunk_len, std::vector<float>(3, 0));

    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        ptcld_chunk[i][j] = ptcld[cloud_cut+i][j];
      }
    }

    // std::cout << cloud_cut << '\n';

    cloud_cut = cloud_cut + cloud_chunk_len;

    // Iterate through chunk of pointcloud
    for(int p = 0; p < cloud_chunk_len; ++p)
    {

      // Position of pointcloud point at index within bounding box
      m_cld_prox = 100;
      for(index i = 0; i < x_pts; ++i)
      {
        for(index j = 0; j < y_pts; ++j)
        {
          for(index k = 0; k < z_pts; ++k)
          {

            prox_2 = (box[i][j][k][0] - ptcld_chunk[p][0])*(box[i][j][k][0] - ptcld_chunk[p][0]) + (box[i][j][k][1] - ptcld_chunk[p][1])*(box[i][j][k][1] - ptcld_chunk[p][1]) + (box[i][j][k][2] - ptcld_chunk[p][2])*(box[i][j][k][2] - ptcld_chunk[p][2]);

            if(prox_2 < m_cld_prox)
            {
              m_cld_prox = prox_2;
              point_in_cloud[0] = int(i);
              point_in_cloud[1] = int(j);
              point_in_cloud[2] = int(k);
            }
          }
        }
      }


      // Direction of ray
      dir_sum = 0;
      for(int i = 0; i < 3; ++i)
      {
        direc[i] = point_in_cloud[i] - origin[i];
        dir_sum = dir_sum + abs(direc[i]);
      }

      // Probability and inds vector setup
      for(int i = 0; i < 3; ++i)
      {
        probs[i] = abs(float(direc[i]) / float(dir_sum));
        inds[i] = i;
      }

      // Unitizing the direction
      for(int i = 0; i < 3; ++i)
      {
        if(direc[i] >= 1)
        {
          direc[i] = 1;
        }
        else if(direc[i] <= -1)
        {
          direc[i] = -1;
        }
        else
        {
          direc[i] = 0;
        }
      }

      // Set point equal to sensor origin
      for(int i = 0; i < 3; ++i)
      {
        pt[i] = origin[i];
      }

      // Stochastic ray-tracing
      std::random_device rd;
      std::mt19937 gen(rd());
      std::discrete_distribution<> distribution(std::begin(probs), std::end(probs));

      while(pt != point_in_cloud)
      {
        f_dist = (point_in_cloud[0] - pt[0])*(point_in_cloud[0] - pt[0]) + (point_in_cloud[1] - pt[1])*(point_in_cloud[1] - pt[1]) + (point_in_cloud[2] - pt[2])*(point_in_cloud[2] - pt[2]);

        pt_ind = distribution(gen);
        pt[pt_ind] = pt[pt_ind] + direc[pt_ind];

        n_dist = (point_in_cloud[0] - pt[0])*(point_in_cloud[0] - pt[0]) + (point_in_cloud[1] - pt[1])*(point_in_cloud[1] - pt[1]) + (point_in_cloud[2] - pt[2])*(point_in_cloud[2] - pt[2]);

        if(n_dist > f_dist)
        {
          pt[pt_ind] = pt[pt_ind] - direc[pt_ind];
        }
        else
        {
          box_miss[pt[0]][pt[1]][pt[2]] = box_miss[pt[0]][pt[1]][pt[2]] + 1;
        }

      }

      // Add hit to box
      box_hit[pt[0]][pt[1]][pt[2]] = box_hit[pt[0]][pt[1]][pt[2]] + 1;

      // Determine unit vector of dir from origin to point
      // max_ax = 0;
      // for(int ax = 0; ax < 3; ++ax)
      // {
      //   dist[ax] = ptcld_chunk[p][ax] - path[o][ax];
      //   if(abs(dist[ax]) > max_ax)
      //   {
      //     max_ax = abs(dist[ax]);
      //   }
      // }
      // for(int ax = 0; ax < 3; ++ax)
      // {
      //   dir[ax] = dist[ax] / max_ax;
      // }
      //
      // mag = std::sqrt(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2]);
      // increment_index = std::floor(mag / resolution);
      // increment = mag / increment_index;

      // Iterate through ray cast
      // for(int i = 0; i < increment_index; ++i)
      // {
      //   // std::cout << '\n' << endl;
      //   // std::cout << "inc ind: "<< i << '\n';
      //   // Create intermittent point
      //   for(int j = 0; j < 3; ++j)
      //   {
      //     pt_x = path[o][0] + (dist[0] / increment_index) * (i + 1);
      //     pt_y = path[o][1] + (dist[1] / increment_index) * (i + 1);
      //     pt_z = path[o][2] + (dist[2] / increment_index) * (i + 1);
      //   }
      //
      //   // Comb bounding box for intersections
      //   for(index x = 0; x < x_pts; ++x)
      //   {
      //     for(index y = 0; y < y_pts; ++y)
      //     {
      //       for(index z = 0; z < z_pts; ++z)
      //       {
      //         // std::cout << "bbox ind: " << k << '\n';
      //         if(i != increment_index-1)
      //         {
      //           prox_2 = (box[x][y][z][0] - pt_x)*(box[x][y][z][0] - pt_x) + (box[x][y][z][1] - pt_y)*(box[x][y][z][1] - pt_y) + (box[x][y][z][2] - pt_z)*(box[x][y][z][2] - pt_z);
      //           if(prox_2 < (resolution*resolution) / 4)
      //           {
      //             for(int h = 0; h < 3; ++h)
      //             {
      //               box_miss[x][y][z] = box_miss[x][y][z] + 1;
      //             }
      //           }
      //         }
      //         else
      //         {
      //           for(int h = 0; h < 3; ++h)
      //           {
      //             box_hit[x][y][z] = box_hit[x][y][z] + 1;
      //           }
      //         }
      //
      //       }
      //     }
      //   }
      //
      // }

    }

    ray_caster::timestamp(start,
                          "end_path_step"
                          );

  }

  ray_caster::timestamp(start,
                        "post_path"
                        );

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

ray_caster::~ray_caster() {}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void ray_caster::timestamp(double start,
                           string checkpt
                           )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int ray_caster::init_zero(int box_len,
                          int **box
                          )
{
  for(int i = 0; i < box_len; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      box[i][j] = 0.;
    }
  }

  return **box;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Free heap space
void ray_caster::int_kill(int pos_len,
                          int **pos
                          )
{
  for(int i = 0; i < pos_len; ++i)
  {
    delete [] pos[i];
  }

  delete [] pos;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Linspace-like vectors
std::vector<float> ray_caster::linspace(float a, float b, int n)
{
  std::vector<float> spaced_vec;
  float step = (b - a) / (n - 1);

  while(a <= b)
  {
    spaced_vec.push_back(a);
    a += step;
  }

  return spaced_vec;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

std::vector<float> ray_caster::g_range(float a, float spacing, float b)
{
  std::vector<float> ranged_vec;
  while(a <= b)
  {
    ranged_vec.push_back(a);
    a += spacing;
  }

  return ranged_vec;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// float ray_caster::vector_dot(const vector *a,const vector *b)
// {
//   return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
// }
//
// void ray_caster::vector_norm(vector *a)
// {
//   float mag = sqrt(vector_dot(a,a));
//   a->x /= mag;
//   a->y /= mag;
//   a->z /= mag;
// }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Takes in vector, returns values above or below zero
// std::vector<float> ray_caster::bifurcate(std::vector<float> spaced_vec, int dir)
// {
//
//   std::sort(spaced_vec.begin(), spaced_vec.end());
//   std::vector<float> bifur_vec;
//
//   if(dir == -1)
//   {
//     for(int i = 0; i < spaced_vec.size(); ++i)
//     {
//       if(spaced_vec[i] > 0)
//       {
//         break;
//       }
//       bifur_vec.push_back(spaced_vec[i]);
//     }
//   }
//   else if(dir == 1)
//   {
//     std::reverse(spaced_vec.begin(), spaced_vec.end());
//     for(int i = 0; i < spaced_vec.size(); ++i)
//     {
//       if(spaced_vec[i] < 0)
//       {
//         break;
//       }
//       bifur_vec.push_back(spaced_vec[i]);
//     }
//   }
//   else
//   {
//     std::cout << "Improper bifurcation dir! Terminating." << '\n';
//     exit(1);
//   }
//
//   return bifur_vec;
// }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Deduces correct box to perform ray casting in based on dir and origin
// int ray_caster::switchboard(vector<float> & dir,
//                             vector<float> & origin
//                             )
// {
//
//
//   return box_no;
// }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//
