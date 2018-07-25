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
  namespace H5
  {
    const H5std_string FILE_NAME( file_name );
    const H5std_string DATASET_NAME( dataset_name );
    const int    NX_SUB = 3;    // hyperslab dimensions
    const int    NY_SUB = 4;
    const int    NX = 7;        // output buffer dimensions
    const int    NY = 7;
    const int    NZ = 3;
    const int    RANK_OUT = 3;

    try
    {
      /*
      * Turn off the auto-printing when failure occurs so that we can
      * handle the errors appropriately
      */
      Exception::dontPrint();
      /*
      * Open the specified file and the specified dataset in the file.
      */
      H5File file( FILE_NAME, H5F_ACC_RDONLY );
      DataSet dataset = file.openDataSet( DATASET_NAME );

      DataSpace dataspace = dataset.getSpace();
      /*
      * Get the number of dimensions in the dataspace.
      */
      int rank = dataspace.getSimpleExtentNdims();
      /*
      * Get the dimension size of each dimension in the dataspace and
      * display them.
      */
      hsize_t dims_out[2];
      int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
      cout << "rank " << rank << ", dimensions " << (unsigned long)(dims_out[0]) << " x " <<(unsigned long)(dims_out[1]) << endl;
      /*
      * Define hyperslab in the dataset; implicitly giving strike and
      * block NULL.
      */
      hsize_t offset[2];   // hyperslab offset in the file
      hsize_t count[2];    // size of the hyperslab in the file
      offset[0] = 1;
      offset[1] = 2;
      count[0] = NX_SUB;
      count[1] = NY_SUB;
      dataspace.selectHyperslab( H5S_SELECT_SET, count, offset );
      /*
      * Define the memory dataspace.
      */
      hsize_t dimsm[3]; /* memory space dimensions */
      dimsm[0] = NX;
      dimsm[1] = NY;
      dimsm[2] = NZ ;
      DataSpace memspace( RANK_OUT, dimsm );
      /*
      * Define memory hyperslab.
      */
      hsize_t offset_out[3]; // hyperslab offset in memory
      hsize_t count_out[3]; // size of the hyperslab in memory
      offset_out[0] = 3;
      offset_out[1] = 0;
      offset_out[2] = 0;
      count_out[0] = NX_SUB;
      count_out[1] = NY_SUB;
      count_out[2] = 1;
      memspace.selectHyperslab( H5S_SELECT_SET, count_out, offset_out );
      /*
      * Read data from hyperslab in the file into the hyperslab in
      * memory and display the data.
      */
      dataset.read( data_out, PredType::NATIVE_INT, memspace, dataspace );
      for (j = 0; j < NX; j++)
      {
        for (i = 0; i < NY; i++)
        {
          cout << data_out[j][i][0] << " ";
        }
      cout << endl;
      }
    }  // end of try block

    // catch failure caused by the H5File operations
    catch( FileIException error )
    {
       error.printError();
       return -1;
    }
    // catch failure caused by the DataSet operations
    catch( DataSetIException error )
    {
       error.printError();
       return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataSpaceIException error )
    {
       error.printError();
       return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataTypeIException error )
    {
       error.printError();
       return -1;
    }
  }
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
