// Reading scan data from H5 files (header)

#ifndef h5_read_hh
#define h5_read_hh

#include <iostream>
#include <cstring>

#include "pt.hh"

#include "H5Cpp.h"

class h5_read
{
  // private:
    // h5_file_opener open_h5;

  public:

    // Constructor with open file inheritance
    // h5_read(std::string file_name) : open_h5() {}

    int sizeup(std::string dataset_name
               )
    {
      using namespace H5;

      const H5std_string FILE_NAME( "RunData.h5" );
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      const H5std_string DATASET( dataset_name );
      DataSet dset = file.openDataSet( DATASET );
      DataSpace dspace = dset.getSpace();
      hsize_t rank;
      hsize_t dims[2];
      rank = dspace.getSimpleExtentDims(dims, NULL);

      file.close();

      return dims[0];
    }


    void data_read_cld(cldpt buffer[], size_t N)
    {
      using namespace H5;

      const H5std_string FILE_NAME( "RunData.h5" );
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      std::string dataset_name = "/cld";
      const H5std_string DATASET_CLD( dataset_name );
      DataSet dset_cld = file.openDataSet( DATASET_CLD );
      DataSpace dspace_cld = dset_cld.getSpace();

      const H5std_string MEMBER_X("x");
      const H5std_string MEMBER_Y("y");
      const H5std_string MEMBER_Z("z");
      const H5std_string MEMBER_IND("scan_index");
      // const H5std_string MEMBER_INTENSITY("intensity_name");

      CompType h5_cldpt_type( sizeof(cldpt) );
      h5_cldpt_type.insertMember(MEMBER_X, 0, PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_Y, sizeof(float), PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_Z, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);
      h5_cldpt_type.insertMember(MEMBER_IND, sizeof(float)+sizeof(float)+sizeof(float), PredType::NATIVE_INT);

      memset(buffer, 0, N);
      dset_cld.read(buffer, h5_cldpt_type);

      file.close();
    }

    void data_read_pose(pose buffer_pose[],
                        rot_mat buffer_rot[],
                        size_t M
                        )
    {
      using namespace H5;

      const H5std_string FILE_NAME( "RunData.h5" );
      H5File file( FILE_NAME, H5F_ACC_RDONLY );

      std::string dataset_name = "/posData";
      const H5std_string DATASET_POSE( dataset_name );
      DataSet dset_pose = file.openDataSet( DATASET_POSE );
      DataSpace dspace_pose = dset_pose.getSpace();

      const H5std_string MEMBER_TX("tx");
      const H5std_string MEMBER_TY("ty");
      const H5std_string MEMBER_TZ("tz");

      CompType h5_pose_type( sizeof(pose) );
      h5_pose_type.insertMember(MEMBER_TX, 0, PredType::NATIVE_FLOAT);
      h5_pose_type.insertMember(MEMBER_TY, sizeof(float), PredType::NATIVE_FLOAT);
      h5_pose_type.insertMember(MEMBER_TZ, sizeof(float)+sizeof(float), PredType::NATIVE_FLOAT);

      memset(buffer_pose, 0, M);
      dset_pose.read(buffer_pose, h5_pose_type);

      const H5std_string MEMBER_R00("ROO");
      const H5std_string MEMBER_R01("R01");
      const H5std_string MEMBER_R02("R02");
      const H5std_string MEMBER_R10("R10");
      const H5std_string MEMBER_R11("R11");
      const H5std_string MEMBER_R12("R12");
      const H5std_string MEMBER_R20("R20");
      const H5std_string MEMBER_R21("R21");
      const H5std_string MEMBER_R22("R22");

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

      memset(buffer_rot, 0, M);
      dset_pose.read(buffer_rot, h5_rot_type);

      file.close();
    }

    ~h5_read()
    {
      // Close H5 file
      // file.close();
    }

};

#endif
