// Data structure for real-space points (pose, pointcloud)

#ifndef pt_hh
#define pt_hh

struct pt
{
  double x, y, z;

  // Equality comparator overload
  bool operator==(const pt& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

};

// Hash template for indexing
namespace std
{
  template<>
    struct hash<pt>
    {
      size_t operator()(const pt& point
                        ) const
      {
        size_t seed = 0;
        spp::hash_combine(seed, point.x);
        spp::hash_combine(seed, point.y);
        spp::hash_combine(seed, point.z);
        return seed;
      }
    };
}

// To read from hdf5
struct cldpt
{
  float x, y, z;
  int scan_index;
};

struct pose
{
  float x, y, z;
};

// Rotation matrix
struct rot_mat
{
  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
};

#endif
