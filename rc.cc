#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <cstring>

#include <vector>
#include <array>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

// Boost Libraries
#include <boost/functional/hash.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// #include <boost/multi_index_container.hpp>
// #include <boost/multi_index/sequenced_index.hpp>
// #include <boost/multi_index/ordered_index.hpp>
// #include <boost/multi_index/hashed_index.hpp>
// #include <boost/multi_index/composite_key.hpp>
// #include <boost/multi_index/identity.hpp>
// #include <boost/multi_index/member.hpp>

// ROS
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include "ros/message_event.h"
#include "rosbag/stream.h"

// TF2 ROS message handling
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// PCL
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

using namespace std;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
template <class T, size_t ROW, size_t COL>
using matrix = std::array<std::array<T, COL>, ROW>;

void pcl_callback(const PointCloud::ConstPtr& msg,
                  std::vector<pt> & pos;
                  )
{
  printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  pos.reserve(msg->width);
  BOOST_FOREACH(const pcl::PointXYZ& PiC, msg->points)
  {
    printf("\t(%f, %f, %f)\n", PiC.x, PiC.y, PiC.z);
    pos.push_back( {PiC.x, PiC.y, PiC.z} );
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

class pose_fetch
{
  public:
    pose_fetch(ros::NodeHandle nh) : tf2_(buffer_),
                                     target_frame_(""),
                                     tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
    {
      point_sub_.subscribe(n_, "_point_stamped", 10);
      tf2_filter_.registerCallback(boost::bind(&pose_fetch::pose_callback, this, _1));
    }

    void pose_callback(const geometry_msgs::PointStampedConstPtr& point_ptr)
    {
      geometry_msgs::PointStamped point_out;
      try
      {
        buffer_.transform(*point_ptr, point_out, target_frame_);

        ROS_INFO("pose(x:%f y:%f z:%f)\n",
                 point_out.point.x,
                 point_out.point.y,
                 point_out.point.z
                 );
      }
      catch(tf2::TransformException &ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
      }
    }

  private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
    tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Simple timestamp function
void timestamp(double start,
               string checkpt
               )
{
  double elapsed;
  elapsed = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "timestamp at " << checkpt << ": " << elapsed << '\n';
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// For storing pointcloud points
struct pt
{
  const float x, y, z;
};

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

// Occupied voxel data
struct occ_data
{
  int hits;
  float probability;
  bool mask;
};

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Non-std square function
float sq(float n
         )
{
  float m = n*n;
  return m;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Faster floor function
int f_floor(float a
            )
{
  return (int)(a + 32768.) - 32768;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Raycasting function; uses discretization to acquire points along ray path
void cast_ray(pt &origin,
              pt &end,
              float resolution,
              vector<pt> & ray
              )
{
  pt dist = {end.x-origin.x, end.y-origin.y, end.z-origin.z};
  float mag = std::sqrt(sq(dist.x) + sq(dist.y) + sq(dist.z));

  // Floor instead of ceil to dampen chance of mistaken marking of endpt as free
  int disc = int(std::ceil(mag/resolution));
  pt inc = {dist.x / disc, dist.y / disc, dist.z / disc};

  // Warn on short ray
  if(disc < 1)
  {
    std::cout << "WARNING: Ray of length less than 1!" << endl;
  }
  // Do not mark endpt as free
  ray.reserve(disc-1);
  for(int i = 0; i < disc-1; ++i)
  {
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i, origin.z + inc.z * i});
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Parse hash tables (maps) to get real-space centers of voxels
void get_vox_cents(std::unordered_map <ind, int> & vox,
                   std::vector<pt> & cents,
                   float resolution
                   )
{
  // Reserve for centers
  cents.reserve(vox.size());

  // Construct map iterator; loop through map
  std::unordered_map <ind, int> ::iterator vox_iterator;
  for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
  {
    cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
  }

}

void get_masked_cents(std::unordered_map <ind, occ_data> & vox,
                      std::vector<pt> & cents,
                      float resolution
                      )
{
  // Reserve for centers
  cents.reserve(vox.size());

  // Construct map iterator; loop through map
  std::unordered_map <ind, occ_data> ::iterator vox_iterator;
  for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
  {
    if(vox_iterator->second.mask)
    {
      cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
    }
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Write voxel centers to file
void write_cents(std::vector<pt> & cents,
                 std::string filename
                 )
{
  std::ofstream file;
  file.open(filename+".txt");
  for(int i = 0; i < cents.size(); ++i)
  {
    file << cents[i].x << ", " << cents[i].y << ", " << cents[i].z << "\n";
  }
  file.close();
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// struct ind_t
// {
//   const int x, y, z;
//   int hits;
//
//   ind_t(const int x, const int y, const int z, int hits):x(x), y(y), z(z), hits(hits){}
// };
//
// typedef boost::multi_index_container<
//   ind_t,
//   boost::multi_index::indexed_by<
//     boost::multi_index::hashed_non_unique<
//       boost::multi_index::composite_key<
//         ind_t,
//         boost::multi_index::member<ind_t, const int, &ind_t::x>,
//         boost::multi_index::member<ind_t, const int, &ind_t::y>,
//         boost::multi_index::member<ind_t, const int, &ind_t::z>
//       >
//     >,
//     boost::multi_index::ordered_unique<
//       ind_t,
//       boost::multi_index::member<ind_t, const int, &ind_t::x>
//     >
//   >
// > boost_box;

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{
  std::clock_t start;
  start = std::clock();

  // Init ROS node and instantiate handle
  ros::init(argc, argv, "rc");
  ros::NodeHandle nh;

  std::vector<pt> pcl_scan;

  ros::Subscriber sub = nh.subscribe<PointCloud>("/scan_matched_points2", 100, boost::bind(pcl_callback, _1, pcl_scan));

  pose_fetch pf(nh
                );

  ros::spin();

  std::unordered_map <ind, int> box_free;
  std::unordered_map <ind, int> occ_per_pose;
  std::unordered_map <ind, occ_data> box_occ;
  std::unordered_map <ind, int> box_unknown;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box_free;

  // pt origin = {1.053, -4.234, 0.123};
  // pt end = {2.958, 0.342, -1.001};

  float resolution = 0.1;

  int cloud_cut = 0;
  int cloud_chunk_len = int(floor(ptcld_len / path_len));

  pt origin = {path[path_ind][0], path[path_ind][1], path[path_ind][2]};

  // Build chunk of ptcld
  std::vector< std::vector<float> > ptcld_chunk(cloud_chunk_len, std::vector<float>(3, 0.0f));

  for(int i = 0; i < cloud_chunk_len; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      ptcld_chunk[i][j] = ptcld[cloud_cut+i][j];
    }
  }

  cloud_cut = cloud_cut + cloud_chunk_len;

  for(int cld_ind = 0; cld_ind < cloud_chunk_len; ++cld_ind)
  {

    pt end = {ptcld_chunk[cld_ind][0], ptcld_chunk[cld_ind][1], ptcld_chunk[cld_ind][2]};

    std::vector<pt> ray;
    cast_ray(origin,
             end,
             resolution,
             ray
             );

    for(int i = 0; i < ray.size(); ++i)
    {
      // Mark free space
      int x_ind = f_floor(ray[i].x * (1/resolution));
      int y_ind = f_floor(ray[i].y * (1/resolution));
      int z_ind = f_floor(ray[i].z * (1/resolution));
      ++box_free[ {x_ind, y_ind, z_ind} ];
    }

    // Ensure that destructor is called on ray vector
    vector<pt>().swap(ray);

    // Mark occupied space in temporary map
    int x_ind = f_floor(end.x * (1/resolution));
    int y_ind = f_floor(end.y * (1/resolution));
    int z_ind = f_floor(end.z * (1/resolution));
    ++occ_per_pose[ {x_ind, y_ind, z_ind} ];

  }

  // Update permanent occupancy map

  std::unordered_map <ind, int> ::iterator it_pp;
  for(it_pp = occ_per_pose.begin(); it_pp != occ_per_pose.end(); ++it_pp)
  {
    ind cpt = {it_pp->first.x, it_pp->first.y, it_pp->first.z};
    if(box_occ.count(cpt) == 0)
    {
      // box_occ[cpt].discovery = path_ind;
      box_occ[cpt].mask = false;
    }
    ++box_occ[cpt].hits;
    box_occ[cpt].probability = (float)box_occ[cpt].hits / (float)(path_ind+1);

  }

  float mean_probability = 0.0f;
  std::unordered_map <ind, occ_data> ::iterator it_prob;
  for(it_prob = box_occ.begin(); it_prob != box_occ.end(); ++it_prob)
  {
    mean_probability = mean_probability + it_prob->second.probability;
  }

  mean_probability = mean_probability / box_occ.size();
  // std::cout << mean_probability << '\n';

  // Reset temporary occupancy map
  occ_per_pose.clear();

  // Cull occupied space according to occupancy probability
  std::unordered_map <ind, occ_data> ::iterator it_cull;
  float threshold = 1.5f;
  for(it_cull = box_occ.begin(); it_cull != box_occ.end(); ++it_cull)
  {
    ind cpt = {it_cull->first.x, it_cull->first.y, it_cull->first.z};
    if((box_occ[cpt].probability > (threshold * mean_probability)))
    {
      box_occ[cpt].mask = true;
    }
    if((box_occ[cpt].probability < (threshold * mean_probability)))
    {
      box_occ[cpt].mask = false;
    }
  }

  // Update unkown voxels
  box_unknown.clear();

  std::unordered_map <ind, occ_data> ::iterator it_bbx;

  int min_x = box_occ.begin()->first.x;
  int max_x = box_occ.begin()->first.x;
  int min_y = box_occ.begin()->first.y;
  int max_y = box_occ.begin()->first.y;
  int min_z = box_occ.begin()->first.z;
  int max_z = box_occ.begin()->first.z;

  for(it_bbx = box_occ.begin(); it_bbx != box_occ.end(); ++it_bbx)
  {
    if(box_occ[ {it_bbx->first.x, it_bbx->first.y, it_bbx->first.z} ].mask)
    {
      ind cpt = {it_bbx->first.x, it_bbx->first.y, it_bbx->first.z};
      if(box_occ[cpt].probability > (3.0f * mean_probability))
      {
        int x_v = cpt.x;
        int y_v = cpt.y;
        int z_v = cpt.z;
        if(x_v < min_x) { min_x = x_v; } else if (x_v > max_x) { max_x = x_v; }
        if(y_v < min_y) { min_y = y_v; } else if (y_v > max_y) { max_y = y_v; }
        if(z_v < min_z) { min_z = z_v; } else if (z_v > max_z) { max_z = z_v; }
      }
    }
  }

  std::cout << "occupied voxels before: " << box_occ.size() << '\n';

  // Iterate through bounding box
  for(int x_i = min_x; x_i < max_x; ++x_i)
  {
    for(int y_i = min_y; y_i < max_y; ++y_i)
    {
      for(int z_i = min_z; z_i < max_z; ++z_i)
      {
        if(box_free.count( {x_i, y_i, z_i} ) == 0)
        {
          // if(!box_occ[ {x_i, y_i, z_i} ].mask)
          if(box_occ.count( {x_i, y_i, z_i} ) == 0)
          {
            // Store unknown voxel indecies
            ++box_unknown[ {x_i, y_i, z_i} ];
            // std::cout << "(" << x_i << ", " << y_i << ", " << z_i << ")" << '\n';
          }
        }
      }
    }
  }

  std::cout << "occupied voxels after: " << box_occ.size() << '\n';


  std::cout << "free voxels: " << box_free.size() << '\n';
  std::cout << "occupied voxels: " << box_occ.size() << '\n';
  std::cout << "unknown voxels: " << box_unknown.size() << '\n';

  std::vector<pt> cents_occ;
  std::vector<pt> cents_free;
  std::vector<pt> cents_unknown;

  get_masked_cents(box_occ,
                   cents_occ,
                   resolution
                   );

  get_vox_cents(box_free,
                cents_free,
                resolution
                );

  get_vox_cents(box_unknown,
                cents_unknown,
                resolution
                );

  // for(int i = 0; i < cents_occ.size(); ++i)
  // {
  //    std::cout << "(" << cents_occ[i].x << ", " << cents_occ[i].y << ", " << cents_occ[i].z << ")" << '\n';
  // }

  write_cents(cents_occ,
              "occupied"
              );

  write_cents(cents_free,
              "free"
              );

  write_cents(cents_unknown,
              "unknown"
              );

  timestamp(start,
            "end"
            );

  return 0;
}
