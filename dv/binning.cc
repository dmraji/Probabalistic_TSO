#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <functional>
#include <cmath>
#include <ctime>

#include <boost/functional/hash.hpp>
// #include <boost/unordered_map.hpp>

using namespace std;

void timestamp(double start,
               string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

struct pt
{
  const float x, y;
};

struct ind
{
  const int x, y;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

float sq(float n
         )
{
  float m = n*n;
  return m;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

std::vector<pt> cast_ray(pt origin,
                         pt end,
                         float resolution
                         )
{
  pt dist = {end.x-origin.x, end.y-origin.y};
  float mag = std::sqrt(sq(dist.x) + sq(dist.y));
  int disc = int(std::ceil(mag/resolution));
  pt inc = {dist.x / disc, dist.y / disc};

  std::vector<pt> ray;
  for(int i = 0; i < disc; ++i)
  {
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i});
  }

  return ray;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Unordered_map templates

// Hash template
class hasher
{
  public:
    size_t operator()(const ind& index
                      ) const
    {
      size_t seed = 0;
      boost::hash_combine(seed, index.x);
      boost::hash_combine(seed, index.y);
      return seed;
    }
};

// Key equivalence template
class key_equal
{
  public:
    bool operator()(const ind& index1,
                    const ind& index2
                    ) const
    {
      return (index1.x == index2.x) && (index1.y == index2.y);
    }
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  std::clock_t start;
  start = std::clock();

  unordered_multimap <ind, pt, hasher, key_equal> box;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box_boost;

  pt origin = {1.053, -4.234};
  pt end = {2.958, 0.342};

  float resolution = 0.1;

  std::vector<pt> ray = cast_ray(origin,
                                 end,
                                 resolution
                                 );

  for(int i = 0; i < ray.size(); ++i)
  {
    int x_ind = (ray[i].x * (1/resolution));
    int y_ind = (ray[i].y * (1/resolution));
    box.insert( { {x_ind, y_ind}, ray[i] } );
    // box_boost.insert( { {x_ind, y_ind}, {ray[i].x, ray[i].y} } );

  }

  unordered_multimap <ind, pt, hasher, key_equal> :: iterator it;

  cout << "Unordered multimap contains: " << endl;
  for (it = box.begin(); it != box.end(); ++it)
  {

    std::cout << "(" << it->first.x << ", " << it->first.y << " : " << it->second.x << ", " << it->second.y << ")" << endl;
  }

  // boost::unordered_multimap<ind, pt, boost::hash<ind>>::iterator it;
  // for (it=box_boost.begin(); it!=box_boost.end(); ++it)
  // {
  //   std::cout << it->first <<", " << it->second << std::endl;
  // }

  timestamp(start,
            "end"
            );

  return 0;
}
