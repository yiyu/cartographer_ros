/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <time.h>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>


#include "halo_node_main.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(bag_filenames, "", "Comma-separated list of bags to process.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish the transforms from the bag.");
DEFINE_string(pbstream_filename, "",
              "If non-empty, filename of a pbstream to load.");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");

DEFINE_string(halo_filename, "", "halo record data file");
DEFINE_string(slam_type, "",
              "true/false true:use halo file ,false use rosbag file ");

namespace cartographer_ros {
namespace {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;

int slam_type = false;
//james
//void Run(const std::vector<string>& bag_filenames) {
void Run(const std::string& bag_filenames) {
  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  if (!FLAGS_urdf_filename.empty()) {
    urdf_transforms =
        ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  }

  tf_buffer.setUsingDedicatedThread(true);

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;
  Node node(node_options, &tf_buffer);
  if (!FLAGS_pbstream_filename.empty()) {
    // TODO(jihoonl): LoadMap should be replaced by some better deserialization
    // of full SLAM state as non-frozen trajectories once possible
    node.LoadMap(FLAGS_pbstream_filename);
  }

  std::unordered_set<string> expected_sensor_ids;
  for (const string& topic : node.ComputeDefaultTopics(trajectory_options)) {
    CHECK(expected_sensor_ids.insert(node.node_handle()->resolveName(topic))
              .second);
  }

  ::ros::Publisher tf_publisher =
      node.node_handle()->advertise<tf2_msgs::TFMessage>(
          kTfTopic, kLatestOnlyPublisherQueueSize);

  ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  ::ros::Publisher clock_publisher =
      node.node_handle()->advertise<rosgraph_msgs::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);

  if (urdf_transforms.size() > 0) {
    static_tf_broadcaster.sendTransform(urdf_transforms);
  }

  ros::AsyncSpinner async_spinner(kSingleThreaded);
  async_spinner.start();
  rosgraph_msgs::Clock clock;
  auto clock_republish_timer = node.node_handle()->createWallTimer(
      ::ros::WallDuration(kClockPublishFrequencySec),
      [&clock_publisher, &clock](const ::ros::WallTimerEvent&) {
        clock_publisher.publish(clock);
      },
      false /* oneshot */, false /* autostart */);
      
      //james

      const string& bag_filename = bag_filenames;
      /*
      for (const string& bag_filename : bag_filenames) {
      if (!::ros::ok()) {
        break;
      }*/

      const int trajectory_id =
          node.AddOfflineTrajectory(expected_sensor_ids, trajectory_options);

    //james add
    if(slam_type == 1) 
    {  

       simulate_slam2(node,trajectory_id,bag_filename,clock_publisher,expected_sensor_ids);
    }else if (slam_type ==2)
    {
     
      simulate_imu_slam(node,trajectory_id,bag_filename,clock_publisher,expected_sensor_ids);
    }else
    {
  //
      bool bRecord = false;
      if(!FLAGS_halo_filename.empty() && FLAGS_halo_filename != "none")
      {
        output.open(FLAGS_halo_filename.c_str(),std::ios::out|std::ios::trunc|std::ios::binary);
        std::string strImu = FLAGS_halo_filename + ".txt";
        imu_output.open(strImu.c_str(),std::ios::out|std::ios::trunc);
    //    ROS_INFO(">>>>>>>>>>>>>>>>james::open halo file:%s \n",FLAGS_halo_filename.c_str());
        bRecord = true;
      }

      rosbag::Bag bag;
      bag.open(bag_filename, rosbag::bagmode::Read);
      rosbag::View view(bag);
      const ::ros::Time begin_time = view.getBeginTime();
      const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();

      // We make sure that tf_messages are published before any data messages, so
      // that tf lookups always work and that tf_buffer has a small cache size -
      // because it gets very inefficient with a large one.
      std::deque<rosbag::MessageInstance> delayed_messages;
      for (const rosbag::MessageInstance& msg : view) {
        if (!::ros::ok()) {
          break;
        }

        if (FLAGS_use_bag_transforms && msg.isType<tf2_msgs::TFMessage>()) {
          auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
          tf_publisher.publish(tf_message);

          for (const auto& transform : tf_message->transforms) {
            try {
              tf_buffer.setTransform(transform, "unused_authority",
                                     msg.getTopic() == kTfStaticTopic);
            } catch (const tf2::TransformException& ex) {
              LOG(WARNING) << ex.what();
            }
          }
        }

        while (!delayed_messages.empty() &&
               delayed_messages.front().getTime() <
                   msg.getTime() - ::ros::Duration(1.)) {
          const rosbag::MessageInstance& delayed_msg = delayed_messages.front();
          const string topic = node.node_handle()->resolveName(
              delayed_msg.getTopic(), false /* resolve */);
          if (delayed_msg.isType<sensor_msgs::LaserScan>()) {
            node.HandleLaserScanMessage(
                trajectory_id, topic,
                delayed_msg.instantiate<sensor_msgs::LaserScan>());
          }
          if (delayed_msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
            node.HandleMultiEchoLaserScanMessage(
                trajectory_id, topic,
                delayed_msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
          }
          if (delayed_msg.isType<sensor_msgs::PointCloud2>()) 
          {
              if(bRecord)
              {
                 boost::shared_ptr<sensor_msgs::PointCloud2> point_cloud_msg = delayed_msg.instantiate<sensor_msgs::PointCloud2>();
         
                 const cartographer::common::Time time = FromRos(point_cloud_msg->header.stamp);
                  int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
                 // std::cout << ">james:poitcloud::" << " timestamp:"  <<  point_cloud_msg->header.stamp << " in64_time:"<< uts_timestamp << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
                  write_halo_file(output,delayed_msg.getTopic(),point_cloud_msg);
                  //LOG_EVERY_N(INFO, 1) << ">james:PointCloud2::iCount:"<< iCount << " timestamp:"  <<  point_cloud_msg->header.stamp <<" topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic();
                  //ROS_INFO(">james:PointCloud2::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
              }
              node.HandlePointCloud2Message(
                trajectory_id, topic,
                delayed_msg.instantiate<sensor_msgs::PointCloud2>());
          }

          if (delayed_msg.isType<sensor_msgs::Imu>()) 
          {
             boost::shared_ptr<sensor_msgs::Imu> imu_msg = delayed_msg.instantiate<sensor_msgs::Imu>();
            if(bRecord)
            { 
               write_halo_file(output,imu_msg);
               const cartographer::common::Time time = FromRos(imu_msg->header.stamp);
               int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
               std::cout << ">james:         Imu::" << " timestamp:"  <<  imu_msg->header.stamp << " in64_time:"<< uts_timestamp << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
            }
            node.HandleImuMessage(trajectory_id, topic,
                                  delayed_msg.instantiate<sensor_msgs::Imu>());
          }

          if (delayed_msg.isType<nav_msgs::Odometry>()) {
            node.HandleOdometryMessage(
                trajectory_id, topic,
                delayed_msg.instantiate<nav_msgs::Odometry>());
          }
          clock.clock = delayed_msg.getTime();
          clock_publisher.publish(clock);

          LOG_EVERY_N(INFO, 100000)
              << "Processed " << (delayed_msg.getTime() - begin_time).toSec()
              << " of " << duration_in_seconds << " bag time seconds...";

          delayed_messages.pop_front();
        }

        const string topic =
            node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
        if (expected_sensor_ids.count(topic) == 0) {
          continue;
        }
        delayed_messages.push_back(msg);
      }

      bag.close();
      node.FinishTrajectory(trajectory_id);
    //}

    if(bRecord)
    {
      output.close();
      imu_output.close();
       ROS_INFO("................................james:close halo file....................."); 
    }
  } //halo
  // Ensure the clock is republished after the bag has been finished, during the
  // final optimization, serialization, and optional indefinite spinning at the
  // end.
  clock_republish_timer.start();
  node.RunFinalOptimization();

  const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
  const double wall_clock_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                start_time)
          .count();

  LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
  timespec cpu_timespec = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
  LOG(INFO) << "Elapsed CPU time: "
            << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
  rusage usage;
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif

  if (::ros::ok()) {
    const string output_filename = bag_filenames.front() + ".pbstream";
    LOG(INFO) << "Writing state to '" << output_filename << "'...";
    node.SerializeState(output_filename);
  }
  if (FLAGS_keep_running) {
    ::ros::waitForShutdown();
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {

   ROS_INFO(">>>>>>>>>>>>>START HALO SLAM DEMO  <<<<<<<<<<<<<\n");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  CHECK(!FLAGS_slam_type.empty())
      << "-slam_type is missing.";

  if(FLAGS_slam_type == "1")
    cartographer_ros::slam_type =1;
  else if (FLAGS_slam_type == "2")
  {
      cartographer_ros::slam_type =2;
  }else
    cartographer_ros::slam_type = 0;
  ::ros::init(argc, argv, "cartographer_halo_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
//james
 // cartographer_ros::Run(cartographer_ros::SplitString(FLAGS_bag_filenames, ','));
  cartographer_ros::Run(FLAGS_bag_filenames);

  ::ros::shutdown();
}
