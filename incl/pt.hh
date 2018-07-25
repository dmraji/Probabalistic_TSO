// Data structure for real-space points (pose, pointcloud)

#ifndef pt_hh
#define pt_hh

struct pt
{
  const float x, y, z;
};

// To read from hdf5
struct pt_a
{
  float x, y, z;
};

// Rotation matrix
struct rot_mat
{
  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
};

#endif
