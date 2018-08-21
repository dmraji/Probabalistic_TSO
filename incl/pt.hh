// Data structure for real-space points (pose, pointcloud)

#ifndef pt_hh
#define pt_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct pt
{
  const float x, y, z;

  // Intensity from LiDAR scan
  const float intensity;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct pt_occ
{
  const float x, y, z;

  // Occupancy probability
  const float probability;

  // Intensity from LiDAR scan
  const float intensity;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// To read from hdf5
struct cldpt
{
  float x, y, z;

  // Intensity from LiDAR scan
  float intensity;

  // Index that identifies like scan-points
  int scan_index;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Pose data
struct pose
{
  float x, y, z;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Rotation matrix data
struct rot_mat
{
  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

#endif
