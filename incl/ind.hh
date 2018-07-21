#ifndef ind_hh
#define ind_hh

// Indexing struct for point binning, mapping
struct ind
{
  const int x, y, z;

  // Equality comparator overload for unordered_map
  bool operator==(const ind& other
                  ) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }
};

// Hash template for ind struct
namespace std
{
  template<>
    struct hash<ind>
    {
      size_t operator()(const ind& index
                        ) const
      {
        size_t seed = 0;
        boost::hash_combine(seed, index.x);
        boost::hash_combine(seed, index.y);
        boost::hash_combine(seed, index.z);
        return seed;
      }
    };
}

#endif
