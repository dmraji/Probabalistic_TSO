link_directories(/usr/lib/x86_64-linux-gnu/hdf5/serial)
include_directories(/usr/include/hdf5/serial)
target_link_libraries(${name} hdf5 hdf5_cpp)

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"


struct pt_a
{
  float x, y, z;
};



void h5_read(std::string file_name,
             std::string dataset_name
             )
{
  using namespace H5;

  const H5std_string FILE_NAME( file_name );
  const H5std_string DATASET_NAME( dataset_name );
  // const int    NX_SUB = 3;    // hyperslab dimensions
  // const int    NY_SUB = 4;
  // const int    NX = 7;        // output buffer dimensions
  // const int    NY = 7;
  // const int    NZ = 3;
  // const int    RANK_OUT = 3;

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

  pt_a *data_h5 = new pt_a[40987774];
  memset(data_h5, 0, 40987774);
  dataset.read(data_h5, h5_pt_type);

  for(int i = 0; i < 10000; ++i)
  {
    std::cout << i << ": " << data_h5[i].x << '\n';
  }

    // try
    // {
    //   /*
    //   * Turn off the auto-printing when failure occurs so that we can
    //   * handle the errors appropriately
    //   */
    //   Exception::dontPrint();
    //   /*
    //   * Open the specified file and the specified dataset in the file.
    //   */
    //   H5File file( FILE_NAME, H5F_ACC_RDONLY );
    //   DataSet dataset = file.openDataSet( DATASET_NAME );
    //
    //   DataSpace dataspace = dataset.getSpace();
    //   /*
    //   * Get the number of dimensions in the dataspace.
    //   */
    //   int rank = dataspace.getSimpleExtentNdims();
    //   /*
    //   * Get the dimension size of each dimension in the dataspace and
    //   * display them.
    //   */
    //   hsize_t dims_out[2];
    //   int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
    //   cout << "rank " << rank << ", dimensions " << (unsigned long)(dims_out[0]) << " x " <<(unsigned long)(dims_out[1]) << endl;
    //   /*
    //   * Define hyperslab in the dataset; implicitly giving strike and
    //   * block NULL.
    //   */
    //   hsize_t offset[2];   // hyperslab offset in the file
    //   hsize_t count[2];    // size of the hyperslab in the file
    //   offset[0] = 1;
    //   offset[1] = 2;
    //   count[0] = NX_SUB;
    //   count[1] = NY_SUB;
    //   dataspace.selectHyperslab( H5S_SELECT_SET, count, offset );
    //   /*
    //   * Define the memory dataspace.
    //   */
    //   hsize_t dimsm[3]; /* memory space dimensions */
    //   dimsm[0] = NX;
    //   dimsm[1] = NY;
    //   dimsm[2] = NZ ;
    //   DataSpace memspace( RANK_OUT, dimsm );
    //   /*
    //   * Define memory hyperslab.
    //   */
    //   hsize_t offset_out[3]; // hyperslab offset in memory
    //   hsize_t count_out[3]; // size of the hyperslab in memory
    //   offset_out[0] = 3;
    //   offset_out[1] = 0;
    //   offset_out[2] = 0;
    //   count_out[0] = NX_SUB;
    //   count_out[1] = NY_SUB;
    //   count_out[2] = 1;
    //   memspace.selectHyperslab( H5S_SELECT_SET, count_out, offset_out );
    //   /*
    //   * Read data from hyperslab in the file into the hyperslab in
    //   * memory and display the data.
    //   */
    //   dataset.read( data_out, PredType::NATIVE_INT, memspace, dataspace );
    //   for (j = 0; j < NX; j++)
    //   {
    //     for (i = 0; i < NY; i++)
    //     {
    //       cout << data_out[j][i][0] << " ";
    //     }
    //   cout << endl;
    //   }
    // }  // end of try block
    //
    // // catch failure caused by the H5File operations
    // catch( FileIException error )
    // {
    //    error.printError();
    //    return -1;
    // }
    // // catch failure caused by the DataSet operations
    // catch( DataSetIException error )
    // {
    //    error.printError();
    //    return -1;
    // }
    // // catch failure caused by the DataSpace operations
    // catch( DataSpaceIException error )
    // {
    //    error.printError();
    //    return -1;
    // }
    // // catch failure caused by the DataSpace operations
    // catch( DataTypeIException error )
    // {
    //    error.printError();
    //    return -1;
    // }
}

h5_read("file", "dataset")
