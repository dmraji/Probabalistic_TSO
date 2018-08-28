// Data structure for real-space points (pose, pointcloud)

#ifndef pt_hh
#define pt_hh

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct pt
{
  double x, y, z;

  //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_

  // Equality comparator overload
  bool operator==(const pt& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

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

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// To read from hdf5
struct cldpt
{
  float x, y, z;
  int scan_index;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct pose
{
  float x, y, z;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Rotation matrix
struct rot_mat
{
  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
};

#endif
