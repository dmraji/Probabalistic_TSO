#include <iostream>

#include <cmath>
#include <vector>

#include "ray_caster.hh"

using namespace std;

ray_caster::ray_caster(float **ptcld,
                       int ptcld_len,
                       float **path,
                       int path_len,
                       float** boundaries
                       )
{

  std::cout << boundaries[0][0] - boundaries[1][0] << '\n';

  // float** box = new float*[]

  int** ptcld_hit = new int*[ptcld_len];
  for(int i = 0; i < ptcld_len; i++)
  {
    ptcld_hit[i] = new int[3];
  }

  int** ptcld_miss = new int*[ptcld_len];
  for(int i = 0; i < ptcld_len; i++)
  {
    ptcld_miss[i] = new int[3];
  }

  for(int o = 0; o < 1; o++)
  {

    std::cout << "hi" << '\n';

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

  std::cout << "hil" << '\n';

}

ray_caster::~ray_caster() {}

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
