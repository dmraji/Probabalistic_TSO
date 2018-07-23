// Data structure for real-space points (pose, pointcloud)

#ifndef pt_hh
#define pt_hh

struct pt
{
  const float x, y, z;

  pt() : x(0.0f),
         y(0.0f),
         z(0.0f) 
         {}
};

#endif
