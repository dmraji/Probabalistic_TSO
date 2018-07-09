#include <fstream>
#include <iostream>
#include <fstream>

#include <string>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

#include <boost/functional/hash.hpp>
// #include <boost/unordered_map.hpp>

using namespace std;

void timestamp(double start,
               string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
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

struct pt
{
  const float x, y, z;
};

struct ind
{
  const int x, y, z;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

float sq(float n
         )
{
  float m = n*n;
  return m;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void cast_ray(pt &origin,
              pt &end,
              float resolution,
              vector<pt> &ray
              )
{
  pt dist = {end.x-origin.x, end.y-origin.y, end.z-origin.z};
  float mag = std::sqrt(sq(dist.x) + sq(dist.y) + sq(dist.z));
  int disc = int(std::ceil(mag/resolution));
  pt inc = {dist.x / disc, dist.y / disc, dist.z / disc};

  ray.reserve(disc);
  for(int i = 0; i < disc; ++i)
  {
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i, origin.z + inc.z});
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Unordered_map templates

// Hash template
class hasher
{
  public:
    size_t operator()(const ind& index
                      ) const
    {
      size_t seed = 0;
      boost::hash_combine(seed, index.x);
      boost::hash_combine(seed, index.y);
      boost::hash_combine(seed, index.z);
      return seed;
    }
};

// Key equivalence template
class key_equal
{
  public:
    bool operator()(const ind& index1,
                    const ind& index2
                    ) const
    {
      return (index1.x == index2.x) && (index1.y == index2.y) && (index1.z == index2.z);
    }
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  std::clock_t start;
  start = std::clock();

  int path_len = 525;
  std::vector< std::vector<float> > path(path_len, std::vector<float>(3, 0));

  int ptcld_len = 14563019;
  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0));

  pos_bin_read("ptcld.bin",
               ptcld);

  timestamp(start,
            "pointcloud");

  pos_bin_read("pos.bin",
               path);

  timestamp(start,
           "path");

  // unordered_multimap <ind, pt, hasher, key_equal> box;
  unordered_map <ind, int, hasher, key_equal> box;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box;

  // pt origin = {1.053, -4.234, 0.123};
  // pt end = {2.958, 0.342, -1.001};

  float resolution = 0.1;

  for(int path_ind = 0; path_ind < 10; ++path_ind)
  {

    pt origin = {path[path_ind][0], path[path_ind][1], path[path_ind][2]};

    int cloud_cut = 0;
    int cloud_chunk_len = int(floor(ptcld_len / path_len));

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

    for(int cld_ind = 0; cld_ind < cloud_chunk_len; ++cld_ind)
    {

      pt end = {ptcld_chunk[cld_ind][0], ptcld_chunk[cld_ind][1], ptcld_chunk[cld_ind][2]};

      std::vector<pt> ray;
      cast_ray(origin,
               end,
               resolution,
               ray
               );

      for(int i = 0; i < ray.size(); ++i)
      {
        int x_ind = (ray[i].x * (1/resolution));
        int y_ind = (ray[i].y * (1/resolution));
        int z_ind = (ray[i].z * (1/resolution));
        // box.insert( { {x_ind, y_ind, z_ind}, ray[i] } );
        ++box[ {x_ind, y_ind, z_ind} ];
        // box_boost.insert( { {x_ind, y_ind}, {ray[i].x, ray[i].y} } );

      }

    }

    timestamp(start,
             "path_pt");

  }

  // unordered_multimap <ind, pt, hasher, key_equal> :: iterator it;

  // cout << "Unordered multimap contains: " << endl;
  // for (it = box.begin(); it != box.end(); ++it)
  // {
  //
  //   std::cout << "(" << it->first.x << ", " << it->first.y << ", " << it->first.z << " : "
  //             << it->second.x << ", " << it->second.y << ", " << it->second.z << ")" << endl;
  // }

  timestamp(start,
            "end"
            );

  return 0;
}
