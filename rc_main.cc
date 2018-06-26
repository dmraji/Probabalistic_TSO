#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>

#include <exception>
#include <ctime>

#include "ray_caster.hh"

using namespace std;

void handle_error(const char* msg)
{
    perror(msg);
    exit(255);
}

void timestamp(double start,
               string checkpt
               )
{
  double elapsed;
  elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Read large pos files into dynamic arrays on the heap
void pos_reader(string fname_str,
                 vector< vector<float> > & pos,
                 string delimiter
                 )
{

  string rwx = "r";

  FILE* f = fopen(fname_str.c_str(), rwx.c_str());
  if (NULL == f)
  {
    printf("Failed to open file! Terminating.");
    exit(1);
  }

  float x, y, z;
  int ln_ctr = 0;
  while(fscanf(f, "%f %f %f\n", &x, &y, &z) != EOF)
  {
    pos[ln_ctr][0] = x;
    pos[ln_ctr][1] = y;
    pos[ln_ctr][2] = z;

    ++ln_ctr;
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

float bnbx(int pos_len,
           vector< vector<float> > & pos,
           float **boundaries
           )
{

  float x_max = -10.;
  float x_min = 10.;
  float y_max = -10.;
  float y_min = 10.;
  float z_max = -10.;
  float z_min = 10.;

  for(int i = 0; i < pos_len; i++)
  {
    // x-coord
    if(pos[i][0] > x_max)
    {
      x_max = pos[i][0];
    }
    else
    {
      if(pos[i][0] < x_min)
      {
        x_min = pos[i][0];
      }
    }

    // y-coord
    if(pos[i][1] > y_max)
    {
      y_max = pos[i][1];
    }
    else
    {
      if(pos[i][1] < y_min)
      {
        y_min = pos[i][1];
      }
    }

    // z-coord
    if(pos[i][2] > z_max)
    {
      z_max = pos[i][2];
    }
    else
    {
      if(pos[i][2] < z_min)
      {
        z_min = pos[i][2];
      }
    }
  }

  boundaries[0][0] = x_min;
  boundaries[1][0] = x_max;
  boundaries[0][1] = y_min;
  boundaries[1][1] = y_max;
  boundaries[0][2] = z_min;
  boundaries[1][2] = z_max;

  return **boundaries;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Free heap space
void pos_kill(int pos_len,
              float **pos
              )
{
  for(int i = 0; i < pos_len; ++i)
  {
    delete [] pos[i];
  }

  delete [] pos;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Main method
int main(int argc, char** argv)
{

  std::clock_t start;
  start = std::clock();

  cout << endl;
  std::cout << "Init main." << '\n';

  int path_len = 525;

  std::vector< std::vector<float> > path(path_len, std::vector<float>(3, 0));

  // float** path = new float*[path_len];
  // for(int i = 0; i < path_len; i++)
  // {
  //   path[i] = new float[3];
  // }

  int ptcld_len = 14563019;

  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0));

  // float** ptcld = new float*[ptcld_len];
  // for(int i = 0; i < ptcld_len; i++)
  // {
  //   ptcld[i] = new float[3];
  // }

  string delim = " ";

  pos_reader("ptcld.txt",
                     ptcld,
                     delim);

  // **ptcld = pos_reader("ptcld.txt",
  //                      ptcld,
  //                      delim);

  timestamp(start,
            "pointcloud");

  pos_reader("pos.txt",
                    path,
                    delim);

  // **path = pos_reader("pos.txt",
  //                     path,
  //                     delim);

  float** boundaries = new float*[2];
  for(int i = 0; i < 2; i++)
  {
    boundaries[i] = new float[3];
  }

  **boundaries = bnbx(ptcld_len,
                      ptcld,
                      boundaries
                      );

  ray_caster(ptcld,
             ptcld_len,
             path,
             path_len,
             boundaries
             );

  // pos_kill(path_len,
  //          path
  //          );
  //
  // pos_kill(ptcld_len,
  //          ptcld
  //          );

  timestamp(start,
            "end");

  return 0;
}
