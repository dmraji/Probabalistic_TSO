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
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Read large pos text files
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

// Read binary pos files
void pos_bin_read(string fname_str,
                  vector< vector<float> > & pos
                  )
{

  float f;
  ifstream fin(fname_str, ios::binary);
  int r_ind = 0;
  int c_ind = 0;
  while(fin.read(reinterpret_cast<char*>(&f), sizeof(float)))
  {
    pos[r_ind][c_ind] = f;
    ++c_ind;
    if(c_ind > 2)
    {
      c_ind = 0;
      ++r_ind;
    }
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Finding max and min elements in multidimensional vectors
template< class T >
struct column_adapter
{
  column_adapter(size_t column) : m_column( column ) {}

  bool operator()(const std::vector< T > & left, const std::vector< T > & right)
  {
    return left.at( m_column ) < right.at( m_column );
  }

  private:
    size_t m_column;
};

void bnbx(int pos_len,
          vector< vector<float> > & pos,
          vector< vector<float> > & boundaries
          )
{
  for(int i = 0; i < 3; ++i)
  {
    const size_t column = i;
    auto biggest = std::max_element(std::begin(pos), std::end(pos), column_adapter< float >( column ) );
    boundaries[1][i] = (*biggest).at( column );

    auto smallest = std::min_element(std::begin(pos), std::end(pos), column_adapter< float >( column ) );
    boundaries[0][i] = (*smallest).at( column );
  }

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

  int ptcld_len = 14563019;
  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0));

  // string delim = " ";

  pos_bin_read("ptcld.bin",
               ptcld);

  // std::cout << ptcld[ptcld_len-1][0] << '\n';

  timestamp(start,
            "pointcloud");

  pos_bin_read("pos.bin",
               path);

  timestamp(start,
           "path");

  // float** boundaries = new float*[2];
  // for(int i = 0; i < 2; i++)
  // {
  //   boundaries[i] = new float[3];
  // }

  std::vector< std::vector<float> > boundaries(2, std::vector<float>(3, 0));

  bnbx(ptcld_len,
       ptcld,
       boundaries
       );

  timestamp(start,
           "bnbx");

  ray_caster(ptcld,
             ptcld_len,
             path,
             path_len,
             boundaries,
             start
             );

  timestamp(start,
            "end");

  return 0;
}
