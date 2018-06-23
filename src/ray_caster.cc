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

  x_pts = 200;
  y_pts = 200;
  z_pts = 20;
  tot_pts = x_pts*y_pts*z_pts;

  float** box = new float*[tot_pts];
  for(int i = 0; i < tot_pts; i++)
  {
    box[i] = new float[3];
  }

  spaced_x = linspace(min_x, max_x, x_pts);
  spaced_y = linspace(min_y, max_y, y_pts);
  spaced_z = linspace(min_z, max_z, z_pts);

  box_ind = 0;
  for(int i = 0; i < x_pts; i++)
  {
    for(int j = 0; j < y_pts; j++)
    {
      for(int k = 0; k < z_pts; k++)
      {
        box[box_ind][0] = spaced_x[i];
        box[box_ind][1] = spaced_y[j];
        box[box_ind][2] = spaced_z[k];

        ++box_ind;
      }
    }
  }

  // Initialize hit tracker for pointcloud
  int** ptcld_hit = new int*[ptcld_len];
  for(int i = 0; i < ptcld_len; i++)
  {
    ptcld_hit[i] = new int[3];
  }

  // Initialize miss tracker for pointcloud
  int** ptcld_miss = new int*[ptcld_len];
  for(int i = 0; i < ptcld_len; i++)
  {
    ptcld_miss[i] = new int[3];
  }

  for(int o = 0; o < 1; o++)
  {



    // for(int p = 0; p < ptcld_len; p++)
    // {
    //
    // }

  }

  int_kill(ptcld_len,
           ptcld_hit
           );

  int_kill(ptcld_len,
           ptcld_miss
           );

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

ray_caster::~ray_caster() {}

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
vector<float> ray_caster::linspace(float a, float b, int n)
{
    vector<float> spaced_vec;
    float step = (b-a) / (n-1);

    while(a <= b)
    {
        array.push_back(a);
        a += step;
    }

    return spaced_vec;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

float ray_caster::bound_build(float **boundaries
                              )
{

}
