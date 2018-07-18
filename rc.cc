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

<<<<<<< HEAD

=======
>>>>>>> df9f5036537704360ebfe330f695098e14b539eb
bool is_zero(float a) { return a == 0; }

// Read binary pos files
void pos_bin_read(string fname_str,
                  vector< vector<float> > & pos
                  )
{

  float f;
  ifstream fin(fname_str, std::ios::in | std::ios::binary);
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
};

struct ind_dated
{
  const int x, y, z;
  int arrival;

  // Equality comparator overload for unordered_map
  bool operator==(const ind& other
                    ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }
}

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
float __int_as_float(int32_t a
                     )
{
  float r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

int32_t __float_as_int(float a
                       )
{
  int32_t r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Superior natural logarithm
float f_logf(float a
             )
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

// Faster floor function
int f_floor(float a
            )
{
  return (int)(a + 32768.) - 32768;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Raycasting function; uses discretization to acquire points along ray path
void cast_ray(pt &origin,
              pt &end,
              float resolution,
              vector<pt> & ray
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

    struct hash<ind_dated>
    {
      size_t operator()(const ind& index
                        ) const
      {
        size_t seed = 0;
        boost::hash_combine(seed, index.x);
        boost::hash_combine(seed, index.y);
        boost::hash_combine(seed, index.z);
        boost::hash_combine(seed, index.arrival);
        return seed;
      }
    }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void get_vox_cents(std::unordered_map <ind, int> & vox,
                   std::vector<pt> & cents,
                   float resolution
                   )
{
  // Reserve for centers
  cents.reserve(vox.size());

  // Construct map iterator; loop through map
  std::unordered_map <ind, int> ::iterator vox_iterator;
  for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
  {
    cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void write_cents(std::vector<pt> & cents,
                 std::string filename
                 )
{
  std::ofstream file;
  file.open(filename+".txt");
  for(int i = 0; i < cents.size(); ++i)
  {
    file << cents[i].x << ", " << cents[i].y << ", " << cents[i].z << "\n";
  }
  file.close();
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// struct ind_t
// {
//   const int x, y, z;
//   int hits;
//
//   ind_t(const int x, const int y, const int z, int hits):x(x), y(y), z(z), hits(hits){}
// };
//
// typedef boost::multi_index_container<
//   ind_t,
//   boost::multi_index::indexed_by<
//     boost::multi_index::hashed_non_unique<
//       boost::multi_index::composite_key<
//         ind_t,
//         boost::multi_index::member<ind_t, const int, &ind_t::x>,
//         boost::multi_index::member<ind_t, const int, &ind_t::y>,
//         boost::multi_index::member<ind_t, const int, &ind_t::z>
//       >
//     >,
//     boost::multi_index::ordered_unique<
//       ind_t,
//       boost::multi_index::member<ind_t, const int, &ind_t::x>
//     >
//   >
// > boost_box;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  std::clock_t start;
  start = std::clock();

  int path_len = 525;
  std::vector< std::vector<float> > path(path_len, std::vector<float>(3, 0.0f));

  int ptcld_len = 14563019;
  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0.0f));

  pos_bin_read("ptcld.bin",
               ptcld);

  timestamp(start,
            "pointcloud");

  pos_bin_read("pos.bin",
               path);

  timestamp(start,
           "path");


  std::unordered_map <ind, int> box_free;
  std::unordered_map <ind, int> occ_per_pose;
<<<<<<< HEAD
  std::unordered_map <ind_dated, int> occ_staging;
=======
  std::unordered_map <ind, int> occ_initial_entries;
>>>>>>> df9f5036537704360ebfe330f695098e14b539eb
  std::unordered_map <ind, int> box_occ;
  std::unordered_map <ind, float> box_occ_prob;
  std::unordered_map <ind, int> box_unknown;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box_free;

  // pt origin = {1.053, -4.234, 0.123};
  // pt end = {2.958, 0.342, -1.001};

  float resolution = 0.1;

  int cloud_cut = 0;
  int cloud_chunk_len = int(floor(ptcld_len / path_len));

  for(int path_ind = 0; path_ind < path_len; ++path_ind)
  {

    pt origin = {path[path_ind][0], path[path_ind][1], path[path_ind][2]};

    // Build chunk of ptcld
    std::vector< std::vector<float> > ptcld_chunk(cloud_chunk_len, std::vector<float>(3, 0.0f));

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
        int x_ind = f_floor(ray[i].x * (1/resolution));
        int y_ind = f_floor(ray[i].y * (1/resolution));
        int z_ind = f_floor(ray[i].z * (1/resolution));
        ++box_free[ {x_ind, y_ind, z_ind} ];
      }

      // Ensure that destructor is called on ray vector
      vector<pt>().swap(ray);

      // Mark occupied space in temporary map
      int x_ind = f_floor(end.x * (1/resolution));
      int y_ind = f_floor(end.y * (1/resolution));
      int z_ind = f_floor(end.z * (1/resolution));
      ++occ_per_pose[ {x_ind, y_ind, z_ind} ];

      // Update occupancy probability
      // box_occ_prob[ {x_ind, y_ind, z_ind} ] = f_logf(box_occ[ {x_ind, y_ind, z_ind} ] / (path_ind + 1));

    }

    // Update permanent occupancy map
    float mean_probability = 0.0f;
    std::unordered_map <ind, int> ::iterator it_pp;
    for(it_pp = occ_per_pose.begin(); it_pp != occ_per_pose.end(); ++it_pp)
    {
<<<<<<< HEAD
      // Need a "staging" map for theoretically occupied voxels
      ind_dated cpt_dated = {it_pp->first.x, it_pp->first.y, it_pp->first.z, path_ind};
      ++occ_staging[cpt_dated];

      // ++box_occ[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ];
      //
      // box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] = (float)(box_occ[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] / (float)(path_ind+1));
      //
      // mean_probability = mean_probability + box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ];

      // std::cout << box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] << '\n';
    }

    // Comb staging map for transfer candidates
    std::unordered_map <ind_d, int> ::iterator it_check;
    for(it_check = occ_staging.begin(); it_check != occ_staging.end(); ++it_check)
    {
      ind_dated cpt_dated = {it_check->first.x, it_check->first.y, it_check->first.z};
      if(occ_staging[cpt_dated] > )
    }


    // mean_probability = mean_probability / occ_per_pose.size();
    //
    // Reset temporary occupancy map
    occ_per_pose.clear();

    // Cull occupied space according to occupancy probability
    // Provide buffer to give time for probability to become reliable;
    // if(path_ind > 150)
    // {
    //
    //   std::unordered_map <ind, float> ::iterator it_cull;
    //   for(it_cull = box_occ_prob.begin(); it_cull != box_occ_prob.end(); ++it_cull)
    //   {
    //     if(box_occ_prob[ {it_cull->first.x, it_cull->first.y, it_cull->first.z} ] < (1.0f *   mean_probability))
    //     {
    //       // std::cout << "erased" << '\n';
    //       box_occ.erase( {it_cull->first.x, it_cull->first.y, it_cull->first.z} );
    //     }
    //   }
    //
    //   // Update unkown voxels
    //   box_unknown.clear();
    //
    //   std::unordered_map <ind, int> ::iterator it_bbx;
    //   int min_x = box_occ.begin()->first.x;
    //   int min_y = box_occ.begin()->first.y;
    //   int min_z = box_occ.begin()->first.z;
    //   int max_x = box_occ.begin()->first.x;
    //   int max_y = box_occ.begin()->first.y;
    //   int max_z = box_occ.begin()->first.z;
    //   for(it_bbx = box_occ.begin(); it_bbx != box_occ.end(); ++it_bbx)
    //   {
    //     if(it_bbx->first.x < min_x) { min_x = it_bbx->first.x; }
    //     if(it_bbx->first.x > max_x) { max_x = it_bbx->first.x; }
    //     if(it_bbx->first.y < min_y) { min_y = it_bbx->first.y; }
    //     if(it_bbx->first.y > max_y) { max_y = it_bbx->first.y; }
    //     if(it_bbx->first.z < min_z) { min_z = it_bbx->first.z; }
    //     if(it_bbx->first.z > max_z) { max_z = it_bbx->first.z; }
    //   }
    //
    //   // Iterate through bounding box
    //   for(int x_i = min_x; x_i < max_x; ++x_i)
    //   {
    //     for(int y_i = min_y; y_i < max_y; ++y_i)
    //     {
    //       for(int z_i = min_z; z_i < max_z; ++z_i)
    //       {
    //         if(box_free.count( {x_i, y_i, z_i} ) == 0)
    //         {
    //           if(box_occ.count( {x_i, y_i, z_i} ) == 0)
    //           {
    //             // Store unknown voxel indecies
    //             ++box_unknown[ {x_i, y_i, z_i} ];
    //             // std::cout << "(" << x_i << ", " << y_i << ", " << z_i << ")" << '\n';
    //           }
    //         }
    //       }
    //     }
    //   }
    //
    // }

    timestamp(start,
              std::to_string(path_ind));

  }

=======
      ++occ_initial_entries[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ];

      // ++box_occ[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ];
      //
      // box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] = (float)(box_occ[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] / (float)(path_ind+1));
      //
      // mean_probability = mean_probability + box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ];

      // std::cout << box_occ_prob[ {it_pp->first.x, it_pp->first.y, it_pp->first.z} ] << '\n';
    }

    mean_probability = mean_probability / occ_per_pose.size();

    // Reset temporary occupancy map
    occ_per_pose.clear();

    // Cull occupied space according to occupancy probability
    // Provide buffer to give time for probability to become reliable;
    if(path_ind > 150)
    {

      std::unordered_map <ind, float> ::iterator it_cull;
      for(it_cull = box_occ_prob.begin(); it_cull != box_occ_prob.end(); ++it_cull)
      {
        if(box_occ_prob[ {it_cull->first.x, it_cull->first.y, it_cull->first.z} ] < (1.0f *   mean_probability))
        {
          // std::cout << "erased" << '\n';
          box_occ.erase( {it_cull->first.x, it_cull->first.y, it_cull->first.z} );
        }
      }

      // Update unkown voxels
      box_unknown.clear();

      std::unordered_map <ind, int> ::iterator it_bbx;
      int min_x = box_occ.begin()->first.x;
      int min_y = box_occ.begin()->first.y;
      int min_z = box_occ.begin()->first.z;
      int max_x = box_occ.begin()->first.x;
      int max_y = box_occ.begin()->first.y;
      int max_z = box_occ.begin()->first.z;
      for(it_bbx = box_occ.begin(); it_bbx != box_occ.end(); ++it_bbx)
      {
        if(it_bbx->first.x < min_x) { min_x = it_bbx->first.x; }
        if(it_bbx->first.x > max_x) { max_x = it_bbx->first.x; }
        if(it_bbx->first.y < min_y) { min_y = it_bbx->first.y; }
        if(it_bbx->first.y > max_y) { max_y = it_bbx->first.y; }
        if(it_bbx->first.z < min_z) { min_z = it_bbx->first.z; }
        if(it_bbx->first.z > max_z) { max_z = it_bbx->first.z; }
      }

      // Iterate through bounding box
      for(int x_i = min_x; x_i < max_x; ++x_i)
      {
        for(int y_i = min_y; y_i < max_y; ++y_i)
        {
          for(int z_i = min_z; z_i < max_z; ++z_i)
          {
            if(box_free.count( {x_i, y_i, z_i} ) == 0)
            {
              if(box_occ.count( {x_i, y_i, z_i} ) == 0)
              {
                // Store unknown voxel indecies
                ++box_unknown[ {x_i, y_i, z_i} ];
                // std::cout << "(" << x_i << ", " << y_i << ", " << z_i << ")" << '\n';
              }
            }
          }
        }
      }

    }

    timestamp(start,
              std::to_string(path_ind));

  }

>>>>>>> df9f5036537704360ebfe330f695098e14b539eb
  // std::unordered_map <ind, float> :: iterator it_fl;
  // cout << "Unordered multimap contains: " << endl;
  // for(it_fl = box_occ_prob.begin(); it_fl != box_occ_prob.end(); ++it_fl)
  // {
  //
  //   std::cout << "(" << it_fl->first.x << ", " << it_fl->first.y << ", " << it_fl->first.z << " : " << it_fl->second << ")" << endl;
  // }

  std::cout << "free voxels: " << box_free.size() << '\n';
  std::cout << "occupied voxels: " << box_occ.size() << '\n';
  std::cout << "unknown voxels: " << box_unknown.size() << '\n';

  std::vector<pt> cents_occ;
  std::vector<pt> cents_free;
  std::vector<pt> cents_unknown;

  get_vox_cents(box_occ,
                cents_occ,
                resolution
                );

  get_vox_cents(box_free,
                cents_free,
                resolution
                );

  if(path_len > 150)
  {
    get_vox_cents(box_unknown,
                  cents_unknown,
                  resolution
                  );
  }

  // for(int i = 0; i < cents_occ.size(); ++i)
  // {
  //    std::cout << "(" << cents_occ[i].x << ", " << cents_occ[i].y << ", " << cents_occ[i].z << ")" << '\n';
  // }

  write_cents(cents_occ,
              "occupied"
              );

  write_cents(cents_free,
              "free"
              );

  if(path_len > 150)
  {
    write_cents(cents_unknown,
                "unknown"
                );
  }

  timestamp(start,
            "end"
            );

  return 0;
}
