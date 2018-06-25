#include <iostream>

#include <cmath>
#include <vector>

#include "ray_caster.hh"

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

ray_caster::ray_caster(float **ptcld,
                       int ptcld_len,
                       float **path,
                       int path_len,
                       float** boundaries
                       )
{

  diff_x = abs(boundaries[0][0] - boundaries[1][0]);
  diff_y = abs(boundaries[0][1] - boundaries[1][1]);
  diff_z = abs(boundaries[0][2] - boundaries[1][2]);

  std::cout << "diff_x: " << diff_x << '\n';
  std::cout << "diff_y: " << diff_y << '\n';
  std::cout << "diff_z: " << diff_z << '\n';

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
  // x_neg = bifurcate(spaced_x, -1);
  // x_pos = bifurcate(spaced_x, 1);
  spaced_y = g_range(min_y, resolution, max_y);
  // y_neg = bifurcate(spaced_y, -1);
  // y_pos = bifurcate(spaced_y, 1);
  spaced_z = g_range(min_z, resolution, max_z);
  // z_neg = bifurcate(spaced_z, -1);
  // z_pos = bifurcate(spaced_z, 1);

  tot_pts = spaced_x.size() * spaced_y.size() * spaced_z.size();
  float** box = new float*[tot_pts];
  for(int i = 0; i < tot_pts; ++i)
  {
    box[i] = new float[3];
  }

  // Organize box pts in chunks of z-space for easier parsing during raycasting
  box_ind = 0;
  for(int k = 0; k < z_pts; ++k)
  {
    for(int i = 0; i < x_pts; ++i)
    {
      for(int j = 0; j < y_pts; ++j)
      {
        box[box_ind][0] = spaced_x[i];
        box[box_ind][1] = spaced_y[j];
        box[box_ind][2] = spaced_z[k];

        ++box_ind;
      }
    }
  }

  // Initialize hit tracker for box
  int** box_hit = new int*[tot_pts];
  for(int i = 0; i < tot_pts; ++i)
  {
    box_hit[i] = new int[3];
  }

  **box_hit = init_zero(tot_pts,
                        box_hit
                        );

  // Initialize miss tracker for box
  int** box_miss = new int*[tot_pts];
  for(int i = 0; i < tot_pts; ++i)
  {
    box_miss[i] = new int[3];
  }

  **box_miss = init_zero(tot_pts,
                         box_miss
                         );

  // Init vars for path iteration
  std::vector<float> dist(3);
  std::vector<float> dir(3);
  cloud_cut = 0;
  cloud_chunk_len = int(floor(ptcld_len / path_len));
  for(int o = 0; o < 1; ++o)
  {
    std::cout << "path_ind: " << o << '\n';
    // Build chunk of ptcld
    float** ptcld_chunk = new float*[cloud_chunk_len];
    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      ptcld_chunk[i] = new float[3];
    }

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

      std::cout << '\n' << '\n' << '\n' << '\n' << '\n' << '\n' << endl;
      std::cout << "ptcld_ind" << p << '\n';

      // Determine unit vector of dir from origin to point
      max_ax = 0;
      for(int ax = 0; ax < 3; ++ax)
      {
        dist[ax] = ptcld_chunk[p][ax] - path[o][ax];
        if(abs(dist[ax]) > max_ax)
        {
          max_ax = abs(dist[ax]);
        }
      }
      for(int ax = 0; ax < 3; ++ax)
      {
        dir[ax] = dist[ax] / max_ax;
      }

      mag = sqrt(pow(dist[0], 2) + pow(dist[1], 2) + pow(dist[2], 2));
      increment_index = floor(mag / resolution);
      increment = mag / increment_index;

      // std::cout << "mag: " << mag << '\n';
      // std::cout << increment_index << '\n';
      // std::cout << increment << '\n';

      // Iterate through ray cast
      for(int i = 0; i < increment_index; ++i)
      {
        std::cout << '\n' << endl;
        std::cout << "inc ind: "<< i << '\n';
        // Create intermittent point
        for(int j = 0; j < 3; ++j)
        {
          pt_x = path[o][0] + (dist[0] / increment_index) * (i + 1);
          pt_y = path[o][1] + (dist[1] / increment_index) * (i + 1);
          pt_z = path[o][2] + (dist[2] / increment_index) * (i + 1);
        }

        // Comb bounding box for intersections
        for(int k = 0; k < tot_pts; ++k)
        {
          // std::cout << "bbox ind: " << k << '\n';
          if(i != increment_index-1)
          {
            prox = sqrt(pow(box[k][0] - pt_x, 2) + pow(box[k][1] - pt_y, 2) + pow(box[k][2] - pt_z, 2));
            if(prox < resolution / 2)
            {
              for(int h = 0; h < 3; ++h)
              {
                box_miss[k][h] = box_miss[k][h] + 1;
              }
            }
          }
          else
          {
            if(prox < resolution / 2)
            {
              for(int h = 0; h < 3; ++h)
              {
                box_hit[k][h] = box_hit[k][h] + 1;
              }
            }
          }

        }

      }

    }

    // Delete pointcloud chunk
    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      delete [] ptcld_chunk[i];
    }
    delete [] ptcld_chunk;

  }

  int_kill(tot_pts,
           box_hit
           );

  int_kill(tot_pts,
           box_miss
           );

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

ray_caster::~ray_caster() {}

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
