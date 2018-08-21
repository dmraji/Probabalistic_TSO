// Reading scan data from H5 files (header)

#ifndef h5_read_hh
#define h5_read_hh

#include <iostream>
#include <cstring>

#include "pt.hh"

// Add following line to .bashrc to ensure H5 libs are found: export CPATH=/usr/include/hdf5/serial/
#include "H5Cpp.h"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

class h5_read
{
  // private:
    // h5_file_opener open_h5;

  public:

    // Constructor with open file inheritance
    // h5_read(std::string file_name) : open_h5() {}

    // Get dimensions of H5 data in given name of desired dataset
    int sizeup(std::string dataset_name
               )
    {
      using namespace H5;

      // H5 lib filename string handling
      const H5std_string FILE_NAME( "RunData.h5" );

      // H5 lib opening of H5 file with options
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      // H5 lib dataset name string handling
      const H5std_string DATASET( dataset_name );

      // H5 lib open the dataset
      DataSet dset = file.openDataSet( DATASET );

      // H5 lib get the dataspace
      DataSpace dspace = dset.getSpace();

      // Analyze dataspace to determine dimensions
      hsize_t rank;
      hsize_t dims[2];
      rank = dspace.getSimpleExtentDims(dims, NULL);

      file.close();

      // Return dimensions of dataspace
      return dims[0];
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    // Read pointcloud data from H5 given prepared array and its size
    void data_read_cld(cldpt buffer[],
                       size_t N
                       )
    {
      using namespace H5;

      // H5 lib filename string handling
      const H5std_string FILE_NAME( "RunData.h5" );

      // H5 lib opening of H5 file with options
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      // H5 lib dataset name string handling
      std::string dataset_name = "/cld";
      const H5std_string DATASET_CLD( dataset_name );

      // H5 lib open the dataset
      DataSet dset_cld = file.openDataSet( DATASET_CLD );

      // H5 lib get the dataspace
      DataSpace dspace_cld = dset_cld.getSpace();

      // H5 lib string handling of members of dataset
      const H5std_string MEMBER_X("x");
      const H5std_string MEMBER_Y("y");
      const H5std_string MEMBER_Z("z");
      const H5std_string MEMBER_INTENSITY("intensity");
      const H5std_string MEMBER_IND("scan_index");

      // Define compound type of data to read from dataset with an alloc size
      CompType h5_cldpt_type( sizeof(cldpt) );

      // Insert members into compound type with proper memory indentation
      h5_cldpt_type.insertMember(MEMBER_X, 0, PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_Y, sizeof(float), PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_Z, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_INTENSITY, sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_IND, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_INT);

      // Read from the dataset given the data array and dataspace size
      memset(buffer, 0, N);
      dset_cld.read(buffer, h5_cldpt_type);

      file.close();
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    // Read pose data from H5 file given prepared arrays for pose and rotation matrices along with their sizes
    void data_read_pose(pose buffer_pose[],
                        rot_mat buffer_rot[],
                        size_t M
                        )
    {
      using namespace H5;

      // H5 lib filename string handling
      const H5std_string FILE_NAME( "RunData.h5" );

      // H5 lib opening of H5 file with options
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      // H5 lib dataset name string handling
      std::string dataset_name = "/posData";
      const H5std_string DATASET_POSE( dataset_name );

      // H5 lib open the dataset
      DataSet dset_pose = file.openDataSet( DATASET_POSE );

      // H5 lib get the dataspace
      DataSpace dspace_pose = dset_pose.getSpace();

      // H5 lib string handling of members of dataset (for pose)
      const H5std_string MEMBER_TX("tx");
      const H5std_string MEMBER_TY("ty");
      const H5std_string MEMBER_TZ("tz");

      // Define compound type of data to read from dataset with an alloc size (for pose)
      CompType h5_pose_type( sizeof(pose) );

      // Insert members into compound type with proper memory indentation (for pose)
      h5_pose_type.insertMember(MEMBER_TX, 0, PredType::NATIVE_FLOAT);
      h5_pose_type.insertMember(MEMBER_TY, sizeof(float), PredType::NATIVE_FLOAT);
      h5_pose_type.insertMember(MEMBER_TZ, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);

      // Read from the dataset given the data array and dataspace size (for pose)
      memset(buffer_pose, 0, M);
      dset_pose.read(buffer_pose, h5_pose_type);

      // H5 lib string handling of members of dataset (for rotmats)
      const H5std_string MEMBER_R00("ROO");
      const H5std_string MEMBER_R01("R01");
      const H5std_string MEMBER_R02("R02");
      const H5std_string MEMBER_R10("R10");
      const H5std_string MEMBER_R11("R11");
      const H5std_string MEMBER_R12("R12");
      const H5std_string MEMBER_R20("R20");
      const H5std_string MEMBER_R21("R21");
      const H5std_string MEMBER_R22("R22");

      // Define compound type of data to read from dataset with an alloc size (for rotmats)
      CompType h5_rot_type( sizeof(rot_mat) );

      // Insert members into compound type with proper memory indentation (for rotmats)
      h5_rot_type.insertMember(MEMBER_R00, 0, PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R01, sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R02, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R10, sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R11, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R12, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R20, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R21, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_rot_type.insertMember(MEMBER_R22, sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);

      // Read from the dataset given the data array and dataspace size (for rotmats)
      memset(buffer_rot, 0, M);
      dset_pose.read(buffer_rot, h5_rot_type);

      file.close();
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    ~h5_read()
    {
      // Close H5 file
      // file.close();
    }

};

#endif
