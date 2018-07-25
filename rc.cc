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

// Boost Libraries
#include <boost/functional/hash.hpp>
// #include <boost/multi_index_container.hpp>
// #include <boost/multi_index/sequenced_index.hpp>
// #include <boost/multi_index/ordered_index.hpp>
// #include <boost/multi_index/hashed_index.hpp>
// #include <boost/multi_index/composite_key.hpp>
// #include <boost/multi_index/identity.hpp>
// #include <boost/multi_index/member.hpp>

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"

// Data structure headers
#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

// Non-member function headers
#include "il_math.hh"
#include "vox_to_cents.hh"
#include "ray_cast.hh"
#include "occ_update.hh"
#include "bbx.hh"

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function
void timestamp(double start,
               std::string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Read binary pos files
void pos_bin_read(std::string fname_str,
                  std::vector<pt> & pos
                  )
{
  float f;
  ifstream fin(fname_str, std::ios::in | std::ios::binary);
  int c_ind = 0;
  std::vector<float> temp;
  while(fin.read(reinterpret_cast<char*>(&f), sizeof(float)))
  {
    temp.push_back(f);
    ++c_ind;
    if(c_ind > 2)
    {
      pos.push_back( {temp[0],
                      temp[1],
                      temp[2]} );
      temp.clear();
      c_ind = 0;
    }
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void h5_read(std::string file_name,
             std::string dataset_name
             )
{
  using namespace H5;

  const H5std_string FILE_NAME( file_name );
  const H5std_string DATASET_NAME( dataset_name );

  H5File file( FILE_NAME, H5F_ACC_RDONLY );
  DataSet dataset = file.openDataSet( DATASET_NAME );

  const H5std_string MEMBER_X("x");
  const H5std_string MEMBER_Y("y");
  const H5std_string MEMBER_Z("z");
  // const H5std_string MEMBER_INTENSITY("intensity_name");
  // const H5std_string MEMBER_RING("ring_name");

  CompType h5_pt_type( sizeof(pt_a) );
  h5_pt_type.insertMember(MEMBER_X, 0, PredType::NATIVE_FLOAT);
  h5_pt_type.insertMember(MEMBER_Y, sizeof(float), PredType::NATIVE_FLOAT);
  h5_pt_type.insertMember(MEMBER_Z, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);

  int cld_pts = 40987774;
  pt_a *data_h5 = new pt_a[cld_pts];
  memset(data_h5, 0, cld_pts);
  dataset.read(data_h5, h5_pt_type);

  // for(int i = 0; i < 10000; ++i)
  // {
  //   std::cout << i << ": " << data_h5[i].x << '\n';
  // }

  const H5std_string DATASET_NAME( "/posData" );
  DataSet dataset = file.openDataSet( DATASET_NAME );

  const H5std_string MEMBER_TX("tx");
  const H5std_string MEMBER_TY("ty");
  const H5std_string MEMBER_TZ("tz");

  const H5std_string MEMBER_R00("ROO");
  const H5std_string MEMBER_R01("R01");
  const H5std_string MEMBER_R02("R02");
  const H5std_string MEMBER_R10("R10");
  const H5std_string MEMBER_R11("R11");
  const H5std_string MEMBER_R12("R12");
  const H5std_string MEMBER_R20("R20");
  const H5std_string MEMBER_R21("R21");
  const H5std_string MEMBER_R22("R22");

  ComptType h5_pt_type( sizeof(pt_a) );
  h5_pt_type.insertMember(MEMBER_TX, 0, PredType::NATIVE_FLOAT);
  h5_pt_type.insertMember(MEMBER_TY, sizeof(float), PredType::NATIVE_FLOAT);
  h5_pt_type.insertMember(MEMBER_TZ, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);

  CompType h5_rot_type( sizeof(rot_mat) );
  h5_rot_type.insertMember(MEMBER_R00, 0, PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R01, sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R02, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R10, sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R11, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R12, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R20, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R21, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
  h5_rot_type.insertMember(MEMBER_R22, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
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
  // Start clock
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
  std::vector<pt> path;
  path.reserve(path_len);

  int ptcld_len = 14563019;
  std::vector<pt> ptcld;
  ptcld.reserve(ptcld_len);

  pos_bin_read("ptcld.bin",
               ptcld
               );

  timestamp(start, "pointcloud");

  pos_bin_read("pos.bin",
               path
               );

  timestamp(start, "path");

  // Hash tables for occupied voxels
  std::unordered_map <ind, free_unk_data> occ_per_pose;
  std::unordered_map <ind, occ_data> box_occ;

  // Hash tables for free and unknown voxels
  std::unordered_map <ind, free_unk_data> box_free;
  std::unordered_map <ind, free_unk_data> box_unknown;

  // Default resolution 10 cm
  float resolution = 0.1;

  int cloud_cut = 0;
  int cloud_chunk_len = int(floor(ptcld_len / path_len));

  for(int path_ind = 0; path_ind < path_len; ++path_ind)
  {

    pt origin = path[path_ind];

    // Build chunk of ptcld
    std::vector<pt> ptcld_chunk;
    ptcld_chunk.reserve(cloud_chunk_len);

    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      ptcld_chunk.push_back(ptcld[cloud_cut+i]);
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

        ++box_free[cind].hits;
      }

      // Mark occupied space in temporary map
      ind cind = { f_floor(end.x * (1/resolution)),
                   f_floor(end.y * (1/resolution)),
                   f_floor(end.z * (1/resolution)) };

      ++occ_per_pose[cind].hits;

      // Ensure that destructor is called on ray vector
      vector<pt>().swap(ray);
    }

    vox_update(occ_per_pose,
               box_occ,
               box_free,
               box_unknown,
               path_ind
               );

    timestamp(start,
              std::to_string(path_ind));

  }

  out_cents writer(box_occ,
                   box_free,
                   box_unknown,
                   resolution
                   );

  timestamp(start,
            "end"
            );

  return 0;
}
