#include <fstream>
#include <iostream>
#include <fstream>
#include <sstream>

#include <string>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <cmath>
#include <ctime>

// Boost
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/functional/hash.hpp>
// #include <boost/unordered_map.hpp>

// ROS
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include "ros/message_event.h"
#include "rosbag/stream.h"

// ROS messages
#include "tso/IntsAndText.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

// TF2 ROS message handling
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
// #include "tf2_msgs/TFMessage.h"

using namespace std;

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

// Read binary pos files
void pos_bin_read(string fname_str,
                  vector< vector<float> > & pos
                  )
{

  float f;
  ifstream fin(fname_str, ios::binary);
  int r_ind = 0;
  int c_ind = 0;
  while(fin.read(reinterpret_cast<char*>(&f), sizeof(float)))
  {
    pos[r_ind][c_ind] = f;
    ++c_ind;
    if(c_ind > 2)
    {
      c_ind = 0;
      ++r_ind;
    }
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// ROS callback for TF2 msgs
void tf2_callback(const ros::MessageEvent< tf2_msgs::TFMessage const > &msg, tf2_ros::Buffer &buf)
{
  // ros::Time now = ros::Time::now();
  // if(now < tf2_ros::last_update_)
  // {
  //   ROS_WARN_STREAM("Detected jump back in time of " << (tf2_ros::last_update_ - now).toSec() << "s. Clearing TF buffer.");
  //   buffer_.clear();
  // }
  // tf2_ros::last_update_ = now;
  // const tf2_msgs::TFMessage& msg_in = *(msg.getConstMessage());
  // std::string authority = msg.getPublisherName(); // lookup the authority
  // for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
  // {
  //   try
  //   {
  //    buf.setTransform(msg_in.transforms[i], authority, false);
  //   }
  //
  //   catch (tf2::TransformException& ex)
  //   {
  //    std::string temp = ex.what();
  //    ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n",  msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(),  temp.c_str());
  //   }
  // }
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = buf.lookupTransform("map", "world", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    // ros::Duration(1.0).sleep();
  }
  // ROS_INFO("hi");
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

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Non-std square function
float sq(float n
         )
{
  float m = n*n;
  return m;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Implementation of CUDA functions to reinterpret casts
float __int_as_float(int32_t a)
{
  float r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

int32_t __float_as_int(float a)
{
  int32_t r;
  memcpy(&r, &a, sizeof(r));
  return r;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Superior natural logarithm
float f_logf (float a)
{
  float m, r, s, t, i, f;
  int32_t e;

  e = (__float_as_int(a) - 0x3f2aaaab) & 0xff800000;
  m = __int_as_float(__float_as_int(a) - e);
  i = (float)e * 1.19209290e-7f; // 0x1.0p-23

  /* m in [2/3, 4/3] */
  f = m - 1.0f;
  s = f * f;

  /* Compute log1p(f) for f in [-1/3, 1/3] */
  r = fmaf(0.230836749f, f, -0.279208571f); // 0x1.d8c0f0p-3, -0x1.1de8dap-2
  t = fmaf(0.331826031f, f, -0.498910338f); // 0x1.53ca34p-2, -0x1.fee25ap-2
  r = fmaf(r, s, t);
  r = fmaf(r, s, f);
  r = fmaf(i, 0.693147182f, r); // 0x1.62e430p-1 // log(2)

  return r;
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Raycasting function; uses discretization to acquire points along ray path
void cast_ray(pt &origin,
              pt &end,
              float resolution,
              vector<pt> &ray
              )
{
  pt dist = {end.x-origin.x, end.y-origin.y, end.z-origin.z};
  float mag = std::sqrt(sq(dist.x) + sq(dist.y) + sq(dist.z));
  int disc = int(std::ceil(mag/resolution));
  pt inc = {dist.x / disc, dist.y / disc, dist.z / disc};

  ray.reserve(disc);
  for(int i = 0; i < disc; ++i)
  {
    ray.push_back({origin.x + inc.x * i, origin.y + inc.y * i, origin.z + inc.z});
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Hash template
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

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

int main(int argc, char **argv)
{

  // Init ROS node and instantiate handle
  ros::init(argc, argv, "tso");
  ros::NodeHandle nh;

  // Instantiate TF2 listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Publisher pub = nh.advertise<std_msgs::String>("rc_out", 100);

  int bagyes = 1;
  if(bagyes == 1)
  {
    // ROS bag
    rosbag::Bag bag;
    // bag.open("data_lidartest.bag", rosbag::bagmode::Read);
    bag.open("/home/dmraji/2018/cw_2/src/tso/2018-07-11-10-38-17.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    // topics.push_back(std::string("velodyne/velodyne_points"));
    topics.push_back(std::string("/tf"));
    // topics.push_back(std::string("/trajectory"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      // tf_listener.subscription_callback(m);

      tf2_msgs::TFMessage::ConstPtr s_0 = m.instantiate<tf2_msgs::TFMessage>();
      tf2_callback(s_0, tf_buffer);

      // if( (m.getTopic() == "/tf") || ("/" + m.getTopic() == "/tf") )
      // {
      //
      //   tf2_msgs::TFMessage::ConstPtr s_0 = m.instantiate<tf2_msgs::TFMessage>();
      //   if(s_0 != NULL)
      //   {
      //     std::cout << s_0->data << std::endl;
      //   }
      // }

      // std::cout << m.getDataType() << '\n';
    }

  }
  else
  {
    ros::Rate rate(10.0);
    while (nh.ok())
    {
      geometry_msgs::TransformStamped transform_stamped;
      try
      {
        transform_stamped = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      rate.sleep();
    }

  }

  // Create publisher
  // ros::Publisher pub = nh.advertise<tso::IntsAndText>("tso", 100);
  // ros::Rate rate(1);

  // Create subscriber
  // Topic: "slam_data", queue_size: 100, <callback>
  // ros::Subscriber sub = nh.subscribe("slam_data", 100, callback);

  // Loop until shutdown
  // while(ros::ok())
  // {
  //
  //   // Create message
  //   tso::IntsAndText message;
  //
  //   // Load in message components
  //   // message.first = a;
  //   // message.second = b;
  //   // message.third = sum;
  //   // message.text = text.str();
  //
  //   // Publish the message
  //   // pub.publish(message);
  //
  //   // Sleep to fix loop rate
  //   // rate.sleep();
  //
  //   // Spin one time to collect subscribed messages and call callbacks for them
  //   ros::spinOnce();
  // }

  std::clock_t start;
  start = std::clock();

  int path_len = 525;
  std::vector< std::vector<float> > path(path_len, std::vector<float>(3, 0.));

  int ptcld_len = 14563019;
  std::vector< std::vector<float> > ptcld(ptcld_len, std::vector<float>(3, 0.));

  pos_bin_read("ptcld.bin",
               ptcld);

  timestamp(start,
            "pointcloud");

  pos_bin_read("pos.bin",
               path);

  timestamp(start,
           "path");

  // unordered_multimap <ind, pt, hasher, key_equal> box;
  unordered_map <ind, int> box;
  // boost::unordered_multimap<ind, pt, boost::hash<ind>> box;

  // pt origin = {1.053, -4.234, 0.123};
  // pt end = {2.958, 0.342, -1.001};

  float resolution = 0.1;

  for(int path_ind = 0; path_ind < 1; ++path_ind)
  {

    pt origin = {path[path_ind][0], path[path_ind][1], path[path_ind][2]};

    int cloud_cut = 0;
    int cloud_chunk_len = int(floor(ptcld_len / path_len));

    // Build chunk of ptcld
    std::vector< std::vector<float> > ptcld_chunk(cloud_chunk_len, std::vector<float>(3, 0.));

    for(int i = 0; i < cloud_chunk_len; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        ptcld_chunk[i][j] = ptcld[cloud_cut+i][j];
      }
    }

    // std::cout << cloud_cut << '\n';

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
        int x_ind = (ray[i].x * (1/resolution));
        int y_ind = (ray[i].y * (1/resolution));
        int z_ind = (ray[i].z * (1/resolution));
        // box.insert( { {x_ind, y_ind, z_ind}, ray[i] } );
        ++box[ {x_ind, y_ind, z_ind} ];
        // box_boost.insert( { {x_ind, y_ind}, {ray[i].x, ray[i].y} } );

      }

    }

    timestamp(start,
             std::to_string(path_ind));

  }

  // unordered_multimap <ind, pt, hasher, key_equal> :: iterator it;

  // cout << "Unordered multimap contains: " << endl;
  // for (it = box.begin(); it != box.end(); ++it)
  // {
  //
  //   std::cout << "(" << it->first.x << ", " << it->first.y << ", " << it->first.z << " : "
  //             << it->second.x << ", " << it->second.y << ", " << it->second.z << ")" << endl;
  // }

  // ROS spin to stall exit
  // ros::spin();

  timestamp(start,
            "end"
            );

  return 0;
}
