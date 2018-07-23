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

#include <typeinfo>

// Boost Libraries
// #include <boost/functional/hash.hpp>

// #include <boost/multi_index_container.hpp>
// #include <boost/multi_index/sequenced_index.hpp>
// #include <boost/multi_index/ordered_index.hpp>
// #include <boost/multi_index/hashed_index.hpp>
// #include <boost/multi_index/composite_key.hpp>
// #include <boost/multi_index/identity.hpp>
// #include <boost/multi_index/member.hpp>

#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

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

inline bool is_zero(float a) { return a == 0; }

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

// Non-std square function
inline float sq(float n
         )
{
  float m = n*n;
  return m;
}

// Faster floor function
inline int f_floor(float a) { return (int)(a + 32768.) - 32768; }

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

// Parse hash tables (maps) to get real-space centers of voxels
template<class T>
  void get_vox_cents(std::unordered_map <ind, T> & vox,
                     std::vector<pt> & cents,
                     float resolution
                     )
  {
    // Reserve for centers
    cents.reserve(vox.size());
    // Construct map iterator
    std::unordered_map <ind, occ_data> ::iterator vox_iterator;

    switch(typeid(T).name())
    {
      case "occ_data":
        for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
        {
          if(vox_iterator->second.mask)
          {
            cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
          }
        }
        break;

      case "free_unk_data":
        for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
        {
          cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
        }
        break;
    }
  }

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Write voxel centers to file
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
  timestamp(start, "start");

  // CONV TO ARGV
  int max_depth = 3;
  std::vector<int> depth_levels;

  for(int i = 0; i < max_depth; ++i)
  {
    depth_levels.push_back(std::pow(2, i));
  }

  int path_len = 525;
  std::vector<pt> path(path_len, {0.0f, 0.0f, 0.0f});

  int ptcld_len = 14563019;
  std::vector<pt> ptcld(ptcld_len, {0.0f, 0.0f, 0.0f});

  pos_bin_read("ptcld.bin",
               ptcld
               );

  timestamp(start, "pointcloud");

  pos_bin_read("pos.bin",
               path
               );

  timestamp(start, "path");

  std::unordered_map <ind, fu_data> box_free;
  std::unordered_map <ind, fu_data> occ_per_pose;
  std::unordered_map <ind, occ_data> box_occ;
  std::unordered_map <ind, fu_data> box_unknown;

  float resolution = 0.1;

  int cloud_cut = 0;
  int cloud_chunk_len = int(floor(ptcld_len / path_len));

  for(int path_ind = 0; path_ind < path_len; ++path_ind)
  {

    pt origin = path[path_ind];

    // Build chunk of ptcld
    std::vector<pt> ptcld_chunk(cloud_chunk_len, {0.0f, 0.0f, 0.0f});

    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      ptcld_chunk[i] = ptcld[cloud_cut+i];
    }

    cloud_cut = cloud_cut + cloud_chunk_len;

    for(int cld_ind = 0; cld_ind < cloud_chunk_len; ++cld_ind)
    {

      pt end = ptcld_chunk[cld_ind];

      std::vector<pt> ray;
      cast_ray(origin,
               end,
               resolution,
               ray
               );

      for(int i = 0; i < ray.size(); ++i)
      {
        // Mark free space
        ind cind = { f_floor(ray[i].x * (1/resolution)),
                     f_floor(ray[i].y * (1/resolution)),
                     f_floor(ray[i].z * (1/resolution)) };

        ++box_free[cind];
      }

      // Mark occupied space in temporary map
      ind cind = { f_floor(end.x * (1/resolution)),
                   f_floor(end.y * (1/resolution)),
                   f_floor(end.z * (1/resolution)) };

      ++occ_per_pose[cind];

      // Ensure that destructor is called on ray vector
      vector<pt>().swap(ray);
    }

    // Update permanent occupancy map
    std::unordered_map <ind, int> ::iterator it_pp;
    for(it_pp = occ_per_pose.begin(); it_pp != occ_per_pose.end(); ++it_pp)
    {
      ind cpt = {it_pp->first.x, it_pp->first.y, it_pp->first.z};
      // if(box_occ.count(cpt) == 0)
      // {
      //   box_occ[cpt].mask = false;
      // }
      ++box_occ[cpt].hits;
      box_occ[cpt].probability = (float)box_occ[cpt].hits / (float)(path_ind+1);
    }

    // Reset temporary occupancy map
    occ_per_pose.clear();

    // Update mean probability based on occupacy
    float mean_probability = 0.0f;
    std::unordered_map <ind, occ_data> ::iterator it_prob;
    for(it_prob = box_occ.begin(); it_prob != box_occ.end(); ++it_prob)
    {
      mean_probability = mean_probability + it_prob->second.probability;
    }

    mean_probability = mean_probability / box_occ.size();

    // Cull occupied space according to occupancy probability
    std::unordered_map <ind, occ_data> ::iterator it_cull;
    float threshold = 1.5f;
    for(it_cull = box_occ.begin(); it_cull != box_occ.end(); ++it_cull)
    {
      ind cpt = {it_cull->first.x, it_cull->first.y, it_cull->first.z};
      if((box_occ[cpt].probability > (threshold * mean_probability)))
      {
        box_occ[cpt].mask = true;
      }
      if((box_occ[cpt].probability < (threshold * mean_probability)))
      {
        box_occ[cpt].mask = false;
      }
    }

    // Update unkown voxels
    box_unknown.clear();

    std::unordered_map <ind, occ_data> ::iterator it_bbx;

    int min_x = box_occ.begin()->first.x;
    int max_x = box_occ.begin()->first.x;
    int min_y = box_occ.begin()->first.y;
    int max_y = box_occ.begin()->first.y;
    int min_z = box_occ.begin()->first.z;
    int max_z = box_occ.begin()->first.z;

    for(it_bbx = box_occ.begin(); it_bbx != box_occ.end(); ++it_bbx)
    {
      if(box_occ[ {it_bbx->first.x, it_bbx->first.y, it_bbx->first.z} ].mask)
      {
        ind cpt = {it_bbx->first.x, it_bbx->first.y, it_bbx->first.z};
        if(box_occ[cpt].probability > (3.0f * mean_probability))
        {
          int x_v = cpt.x;
          int y_v = cpt.y;
          int z_v = cpt.z;
          if(x_v < min_x) { min_x = x_v; } else if (x_v > max_x) { max_x = x_v; }
          if(y_v < min_y) { min_y = y_v; } else if (y_v > max_y) { max_y = y_v; }
          if(z_v < min_z) { min_z = z_v; } else if (z_v > max_z) { max_z = z_v; }
        }
      }
    }

    // Make sure the box has an even number of voxels
    if(abs(max_x - min_x) % 2) == 1) { ++max_x; }
    if(abs(max_y - min_y) % 2) == 1) { ++max_y; }
    if(abs(max_z - min_z) % 2) == 1) { ++max_z; }

    std::cout << "occupied voxels before: " << box_occ.size() << '\n';

    // Iterate through bounding box
    for(int x_i = min_x; x_i <= max_x; ++x_i)
    {
      for(int y_i = min_y; y_i <= max_y; ++y_i)
      {
        for(int z_i = min_z; z_i <= max_z; ++z_i)
        {
          if(box_free.count( {x_i, y_i, z_i} ) == 0)
          {
            // if(!box_occ[ {x_i, y_i, z_i} ].mask)
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



    std::cout << "occupied voxels after: " << box_occ.size() << '\n';

    timestamp(start,
              std::to_string(path_ind));

  }

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

  get_masked_cents(box_occ,
                   cents_occ,
                   resolution
                   );

  get_vox_cents(box_free,
                cents_free,
                resolution
                );

  get_vox_cents(box_unknown,
                cents_unknown,
                resolution
                );

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

  write_cents(cents_unknown,
              "unknown"
              );

  timestamp(start,
            "end"
            );

  return 0;
}
