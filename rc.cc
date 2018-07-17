#include <fstream>
#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <cstring>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

// Boost Libraries
#include <boost/functional/hash.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function
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

// For storing pointcloud points
struct pt
{
  const float x, y, z;
};

// Indexing struct for point binning, mapping
struct ind
{
  const int x, y, z;

  // Equality comparator overload for unordered_map
  bool operator==(const ind& other
                    ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  struct x_ord
  {
    bool operator()(const ind& a, const ind& b
                    ) const
    {
      return a.x < b.x;
    }
  };

  struct y_ord
  {
    bool operator()(const ind& a, const ind& b
                    ) const
    {
      return a.y < b.y;
    }
  };

  struct z_ord
  {
    bool operator()(const ind& a, const ind& b
                    ) const
    {
      return a.z < b.z;
    }
  };
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Non-std square function
float sq(float n
         )
{
  float m = n*n;
  return m;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Implementation of CUDA functions to reinterpret casts
float __int_as_float(int32_t a)
{
  float r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

int32_t __float_as_int(float a)
{
  int32_t r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Superior natural logarithm
float f_logf (float a)
{
  float m, r, s, t, i, f;
  int32_t e;

  e = (__float_as_int(a) - 0x3f2aaaab) & 0xff800000;
  m = __int_as_float(__float_as_int(a) - e);
  i = (float)e * 1.19209290e-7f; // 0x1.0p-23

  /* m in [2/3, 4/3] */
  f = m - 1.0f;
  s = f * f;

  /* Compute log1p(f) for f in [-1/3, 1/3] */
  r = fmaf(0.230836749f, f, -0.279208571f); // 0x1.d8c0f0p-3, -0x1.1de8dap-2
  t = fmaf(0.331826031f, f, -0.498910338f); // 0x1.53ca34p-2, -0x1.fee25ap-2
  r = fmaf(r, s, t);
  r = fmaf(r, s, f);
  r = fmaf(i, 0.693147182f, r); // 0x1.62e430p-1 // log(2)

  return r;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Raycasting function; uses discretization to acquire points along ray path
void cast_ray(pt &origin,
              pt &end,
              float resolution,
              vector<pt> &ray
              )
{
  pt dist = {end.x-origin.x, end.y-origin.y, end.z-origin.z};
  float mag = std::sqrt(sq(dist.x) + sq(dist.y) + sq(dist.z));

  // Floor instead of ceil to dampen chance of mistaken marking of endpt as free
  int disc = int(std::ceil(mag/resolution));
  pt inc = {dist.x / disc, dist.y / disc, dist.z / disc};

  // Warn on short ray
  if(disc < 1)
  {
    std::cout << "WARNING: Ray of length less than 1!" << endl;
  }
  // Do not mark endpt as free
  ray.reserve(disc-1);
  for(int i = 0; i < disc-1; ++i)
  {
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i, origin.z + inc.z * i});
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Hash template for ind struct
namespace std
{
  template<>
    struct hash<ind>
    {
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
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//



//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct ind_t
{
  const int x, y, z;
  int hits;

  ind_t(const int x, const int y, const int z, int hits):x(x), y(y), z(z), hits(hits){}
};

typedef boost::multi_index_container<
  ind_t,
  boost::multi_index::indexed_by<
    boost::multi_index::hashed_non_unique<
      boost::multi_index::composite_key<
        ind_t,
        boost::multi_index::member<ind_t, const int, &ind_t::x>,
        boost::multi_index::member<ind_t, const int, &ind_t::y>,
        boost::multi_index::member<ind_t, const int, &ind_t::z>
      >
    >,
    boost::multi_index::ordered_unique<
      ind_t,
      boost::multi_index::member<ind_t, const int, &ind_t::x>
    >
  >
> boost_box;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{

  std::clock_t start;
  start = std::clock();

  int path_len = 525;
  std::vector< std::vector<float> > path(path_len, std::vector<float>(3, 0.));

  int ptcld_len = 14563019;
  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0.));

  pos_bin_read("ptcld.bin",
               ptcld);

  timestamp(start,
            "pointcloud");

  pos_bin_read("pos.bin",
               path);

  timestamp(start,
           "path");


  std::unordered_map <ind, int> box_free;
  std::unordered_map <ind, int> box_occ;
  std::unordered_map <ind, float> box_occ_prob;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box_free;

  // pt origin = {1.053, -4.234, 0.123};
  // pt end = {2.958, 0.342, -1.001};

  float resolution = 0.1;

  for(int path_ind = 0; path_ind < path_len; ++path_ind)
  {

    pt origin = {path[path_ind][0], path[path_ind][1], path[path_ind][2]};

    int cloud_cut = 0;
    int cloud_chunk_len = int(floor(ptcld_len / path_len));

    // Build chunk of ptcld
    std::vector< std::vector<float> > ptcld_chunk(cloud_chunk_len, std::vector<float>(3, 0.));

    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        ptcld_chunk[i][j] = ptcld[cloud_cut+i][j];
      }
    }

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
        // Mark free space
        int x_ind = (ray[i].x * (1/resolution));
        int y_ind = (ray[i].y * (1/resolution));
        int z_ind = (ray[i].z * (1/resolution));
        ++box_free[ {x_ind, y_ind, z_ind} ];
      }

      // Ensure that destructor is called on ray vector
      vector<pt>().swap(ray);

      // Mark occupied space
      int x_ind = (end.x * (1/resolution));
      int y_ind = (end.y * (1/resolution));
      int z_ind = (end.z * (1/resolution));
      ++box_occ[ {x_ind, y_ind, z_ind} ];

      // Update occupancy probability
      // box_occ_prob[ {x_ind, y_ind, z_ind} ] = f_logf(box_occ[ {x_ind, y_ind, z_ind} ] / (path_ind + 1));
      box_occ_prob[ {x_ind, y_ind, z_ind} ] = f_logf(box_occ[ {x_ind, y_ind, z_ind} ] / (path_ind + 1));

    }

    timestamp(start,
              std::to_string(path_ind));

  }

  std::unordered_map <ind, int> ::iterator it;
  int min_x = box_occ.begin()->first.x;
  int min_y = box_occ.begin()->first.y;
  int min_z = box_occ.begin()->first.z;
  int max_x = box_occ.begin()->first.x;
  int max_y = box_occ.begin()->first.y;
  int max_z = box_occ.begin()->first.z;
  for(it = box_occ.begin(); it != box_occ.end(); ++it)
  {
    if(it->first.x < min_x) { min_x = it->first.x; }
    if(it->first.x > max_x) { max_x = it->first.x; }
    if(it->first.y < min_y) { min_y = it->first.y; }
    if(it->first.y > max_y) { max_y = it->first.y; }
    if(it->first.z < min_z) { min_z = it->first.z; }
    if(it->first.z > max_z) { max_z = it->first.z; }
  }

  std::cout << max_z << '\n';

  std::unordered_map <ind, float> :: iterator it_fl;

  cout << "Unordered multimap contains: " << endl;
  for(it_fl = box_occ_prob.begin(); it_fl != box_occ_prob.end(); ++it_fl)
  {

    std::cout << "(" << it_fl->first.x << ", " << it_fl->first.y << ", " << it_fl->first.z << " : " << it_fl->second << ")" << endl;
  }

  std::cout << box_occ_prob.size() << '\n';

  timestamp(start,
            "end"
            );

  return 0;
}
