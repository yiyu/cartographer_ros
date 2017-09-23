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

#include <csignal>
#include <sstream>
#include <string>
#include <vector>
# include <unistd.h>
# include <stdio.h>
# include <sys/mman.h>
# include <fcntl.h>

#include <boost/algorithm/string.hpp> 
#include <fstream>
#include <iostream>
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"
#include "msg_conversion.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(bag_filenames, "", "Comma-separated list of bags to process.");
DEFINE_string(halo_filename, "", "halo record data file");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_bool(use_bag_transforms, false,
            "Whether to read, use and republish the transforms from the bag.");
DEFINE_string(slam_type, "",
              "true/false true:use halo file ,false use rosbag file ");
namespace cartographer_ros {
namespace {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr int kLatestOnlyPublisherQueueSize = 1;
int slam_type = false;
volatile std::sig_atomic_t sigint_triggered = 0;
std::fstream output;
std::fstream imu_output;
void SigintHandler(int) { sigint_triggered = 1; }


class InstantMsg
{
public:
  int64 time_stamp_;
  void*  base_;
  int type_;
  int offset_;
  int length_;
  //std::string frame_id;
  std::string topic_;
  ros::Time getTime() const
  {
   
    return ToRos(time_stamp_);
  }
  std::string getTopic() const
  {
    return topic_;
  }
  int isType() const
  {
    return type_;
  }


  int parsmsg(int64 time_stamp,void* base,int type,int offset,int length)
  {
  //ROS_INFO(">james:parsmsg:time_stamp:%llu,base:%p,type:%d,offset:%d,length:%d,topic:%s<\n",time_stamp, base,type,offset,length,topic_.c_str());
  
    int len = 0;
    time_stamp_ = time_stamp;
    base_=base;
    type_=type;
    offset_=offset;
    length_ = length;
       
    if(type == 1)
    {
      topic_ = "imu" ;
      len = length;
    }else if(type == 2)
    {
      std::cout << "ERROR: not support message type=2";

    }else if(type == 4)
    {

      len = length;
      unsigned char strLen = 0;
      memcpy(&strLen,(void*)((char*)base_+offset),sizeof(unsigned char));
      offset +=sizeof(unsigned char);
      len += sizeof(unsigned char);
      char cSensor_id[256] ={0};
      memcpy(cSensor_id,(void*)((char*)base_+offset),strLen);
      offset += (int)strLen ;
      len += (int)strLen;
      ::cartographer::transform::Rigid3d sensor_to_tracking;
      memcpy(&sensor_to_tracking,(void*)((char*)base_+offset),sizeof(::cartographer::transform::Rigid3d));
      offset += sizeof(::cartographer::transform::Rigid3d);
      len += sizeof(::cartographer::transform::Rigid3d);
       std::string sensor_id = cSensor_id;
       topic_ = cSensor_id;
     //  std::cout << "======================parsmsg sensor_id:=====================" << topic_ << std::endl;
     
     /*
      if(sensor_id == "points2_2")
      {
        topic_ = "vertical_laser_3d";
      }
      else if(sensor_id == "points2_1" )
      {
        topic_ = "horizontal_laser_3d";
      }*/
    }else
      std::cout << "ERROR: not support message type";
    
    return len;
  }
  
  boost::shared_ptr<sensor_msgs::PointCloud2> getPointCloud2() const
  {
    //ROS_INFO(">james:getPointCloud2:time_stamp:%llu,base:%p,type:%d,offset:%d,length:%d,topic:%s<\n",time_stamp_, base_,type_,offset_,length_,topic_.c_str());
   
     boost::shared_ptr<sensor_msgs::PointCloud2> msg = boost::make_shared<sensor_msgs::PointCloud2>();
      unsigned char strLen = 0;
      int offset = offset_;
      memcpy(&strLen,(void*)((char*)base_+offset),sizeof(unsigned char));
      offset +=sizeof(unsigned char);
      char cSensor_id[256] ={0};
      memcpy(cSensor_id,(void*)((char*)base_+offset),strLen);
      offset += (int)strLen ;
     
      ::cartographer::transform::Rigid3d sensor_to_tracking;
      memcpy(&sensor_to_tracking,(void*)((char*)base_+offset),sizeof(::cartographer::transform::Rigid3d));
      offset += sizeof(::cartographer::transform::Rigid3d);
      cartographer::sensor::proto::PointCloud pts;
      pts.ParseFromArray(((char*)base_+offset),length_);
      cartographer::sensor::PointCloud ranges = cartographer::sensor::ToPointCloud(pts);
      std::string frame_id ="" ;
      std::string sensor_id = cSensor_id;
     /* if(sensor_id == "points2_2")
      {
        sensor_id = "vertical_laser_3d";
        frame_id="vertical_vlp16_link";
      }
      else if(sensor_id == "points2_1" )
      {
        sensor_id = "horizontal_laser_3d";
        frame_id = "horizontal_vlp16_link";
      }*/
        if(topic_ == "horizontal_laser_3d" || topic_ == "points2_1")
          frame_id = "horizontal_vlp16_link";
        else if( sensor_id == "vertical_laser_3d" || topic_ == "points2_2")
          frame_id="vertical_vlp16_link";
     
      *msg = cartographer_ros::ToPointCloud2Message(time_stamp_,frame_id,ranges);
    return msg;
  }

  boost::shared_ptr<sensor_msgs::Imu> getImu() const
  {
    boost::shared_ptr<sensor_msgs::Imu> msg = boost::make_shared<sensor_msgs::Imu>();

      std::string sensor_id = "imu";
      cartographer::sensor::Data::Imu imu;
      memcpy(&imu,(void*)((char*)base_+offset_),length_);
     
      msg->header.stamp = ToRos(::cartographer::common::FromUniversal(time_stamp_));
      msg->header.frame_id = "imu_link";
      msg->header.stamp = ToRos(time_stamp_);
      msg->linear_acceleration.x = imu.linear_acceleration.x();
      msg->linear_acceleration.y =imu.linear_acceleration.y();
      msg->linear_acceleration.z =imu.linear_acceleration.z();
      msg->angular_velocity.x = imu.angular_velocity.x();
      msg->angular_velocity.y =imu.angular_velocity.y();
      msg->angular_velocity.z =imu.angular_velocity.z();
    return msg;
  }
};

void halo_time(uint32_t& sec, uint32_t& nsec)
{
  #if HAS_CLOCK_GETTIME
  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  sec  = start.tv_sec;
  nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  sec  = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
#endif
};

std::vector<string> SplitString(const string& input, const char delimiter) {
  std::stringstream stream(input);
  string token;
  std::vector<string> tokens;
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// TODO(hrapp): This is duplicated in node_main.cc. Pull out into a config
// unit.
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

void write_halo_file( std::fstream& output,const std::string& sensor_id, const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(*msg, pcl_point_cloud);
    cartographer::sensor::PointCloud point_cloud;
    for (const auto& point : pcl_point_cloud) {
      point_cloud.emplace_back(point.x, point.y, point.z);
    }
    unsigned char type = 4;
    cartographer::sensor::proto::PointCloud pts = cartographer::sensor::ToProto(point_cloud);
    unsigned int len = pts.ByteSize();
    output.write((char*)&len,sizeof(unsigned int));

    const cartographer::common::Time time = FromRos(msg->header.stamp);
    int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
    output.write((char*)&uts_timestamp,sizeof(int64));
    output.write((char*)&type,sizeof(unsigned char));
    unsigned char strLen =(unsigned char) (sensor_id.length() + 1);
    output.write((char*)&strLen,sizeof(unsigned char));
    output.write(sensor_id.c_str(),strLen);
    ::cartographer::transform::Rigid3d sensor_to_tracking;
    output.write((char*)&(sensor_to_tracking),sizeof(::cartographer::transform::Rigid3d));
    pts.SerializeToOstream(&output);
}

void write_halo_file( std::fstream& output, const sensor_msgs::Imu::ConstPtr& msg)
{
   //james
    cartographer::sensor::Data::Imu imu;
    //const Eigen::Vector3d& 
    imu.linear_acceleration =  ToEigen(msg->linear_acceleration);
    //const Eigen::Vector3d& 
    imu.angular_velocity =  ToEigen(msg->angular_velocity);
    const cartographer::common::Time time = FromRos(msg->header.stamp);
    int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
    
    std::stringstream  strImu;
    strImu << uts_timestamp;
    strImu << ",";
    strImu << imu.angular_velocity.x() ;
    strImu << ",";
    strImu << imu.angular_velocity.y() ;
    strImu << ",";
    strImu << imu.angular_velocity.z() ;
    strImu << ",";
    strImu << imu.linear_acceleration.x() ;
    strImu << ",";
    strImu << imu.linear_acceleration.y() ;
    strImu << ",";
    strImu << imu.linear_acceleration.z() ;

    unsigned int len = sizeof(imu);
    output.write((char*)&len,sizeof(unsigned int));
    output.write((char*)&uts_timestamp,sizeof(int64));
    unsigned char type = 1;  
    output.write((char*)&type,sizeof(unsigned char));
    output.write((char*)&imu,len);
    imu_output << strImu.str() << std::endl;
}


void simulate_imu_slam( Node& node,const int trajectory_id,const std::string& strData,::ros::Publisher& clock_publisher,std::unordered_set<string>& expected_sensor_ids)
{
   std::ifstream f;
    f.open(strData.c_str(),std::ios::in);
    if( !f.is_open() )
        throw std::domain_error( "Unable to load input file: " + strData );
    std::string line;
    int iCount =0;
    // getline(f,line);
    std::cout << ">>>>>simulate_imu_slam:" << strData <<std::endl;
    int64 last_stamp = 0;
    int64 prev_stamp = 0;
  
    while(!f.eof() ) 
    {
        //f.clear(std::ios::goodbit);
        getline(f,line);
        boost::trim(line);
        std::vector<string> vecValue =SplitString(line, ',');
        if(vecValue.size()<7)
        {
        //  throw std::domain_error( "Unable to parskey : " + line );
          std::cout << "*********Unable to parskey : " <<  line  << "***********"<< std::endl;
          continue;
        }
     //   std::cout  << " keysize:" << vecValue.size()  << " line= " << line << std::endl;
        boost::shared_ptr<sensor_msgs::Imu> msg = boost::make_shared<sensor_msgs::Imu>();
        std::string sensor_id = "imu";
    try
    {
      int64 time_stamp = boost::lexical_cast<int64>(vecValue[0]);
       if(iCount ==0 )
      {
        last_stamp = time_stamp;
        prev_stamp = NowTimestamp();
      }
      else
      {
        int64 now_time =  NowTimestamp();
        while(time_stamp-last_stamp > now_time -prev_stamp)
         {
          usleep(1000);
          //sleep(1);
           now_time =  NowTimestamp();
         } 

        last_stamp = time_stamp;
        prev_stamp = now_time;
      }
      iCount++;
       
      const string topic = node.node_handle()->resolveName(sensor_id);//, false /* resolve */);
   //   std::cout << " $$$test imu key size:" << vecValue.size() << " timestamp:"<< time_stamp << " x:"<< vecValue[1] << " y:"<< vecValue[2] << " z:"<< vecValue[3] << " 1x:"<< vecValue[4] << " 1y:"<< vecValue[5] << " 1z:"<< vecValue[6] << std::endl;
      double accel_x = boost::lexical_cast<double>(vecValue[4]);
       double accel_y = boost::lexical_cast<double>(vecValue[5]);
      double accel_z = boost::lexical_cast<double>(vecValue[6]);
     
      double ang_x = boost::lexical_cast<double>(vecValue[1]);
      double ang_y = boost::lexical_cast<double>(vecValue[2]);
      double ang_z = boost::lexical_cast<double>(vecValue[3]);
     
       msg->header.stamp = ToRos(::cartographer::common::FromUniversal(time_stamp));
       msg->header.frame_id = "imu_link";
       msg->header.stamp = ToRos(time_stamp);
       msg->linear_acceleration.x = accel_x;
       msg->linear_acceleration.y = accel_y;
       msg->linear_acceleration.z = accel_z;
       msg->angular_velocity.x = ang_x;
       msg->angular_velocity.y = ang_y;
       msg->angular_velocity.z = ang_z;
       node.map_builder_bridge()->sensor_bridge(trajectory_id)->HandleImuMessage(topic,msg);
     //  std::cout << ">james:Imu::iCount:"<< iCount << " timestamp:"  <<  time_stamp <<  " topic:" << topic <<  " accel.x:" << accel_x<<  " accel.y:" << accel_y<<  " accel.z:" << accel_z<<  " ang.x:" << ang_x<<  "  ang.y:" << ang_y <<  "  ang.z:" <<ang_z << std::endl;
        rosgraph_msgs::Clock clock;
        clock.clock = ToRos(time_stamp);
        clock_publisher.publish(clock);
        ::ros::spinOnce();
      }
      catch (boost::bad_lexical_cast& e)
      {
          std::cout << "[exception:iCount:"<< iCount << e.what() << " line:" << line << std::endl;
      }

    }
    std::cout << ">>>>>>>>>>>>>>>>>end : simulate_imu_slam <<<<<<<<<<<<<<<<<<<<" << std::endl;

    
}

void simulate_slam2( Node& node,const int trajectory_id,const std::string& strData,::ros::Publisher& clock_publisher,std::unordered_set<string>& expected_sensor_ids)
{
  void *mapped_mem;
  void* start_addr = 0;
  
  FILE* fd = fopen(strData.c_str(), "rb");
  fseek(fd,0,SEEK_END);
  int nFileLen= ftell(fd);
  int sharedFileName = fileno(fd);
  mapped_mem= (char*)mmap(start_addr, nFileLen, PROT_READ,MAP_PRIVATE,sharedFileName,0);
  size_t cur = 0;
  int flength = 20480;
  
  std::cout << "laser data size:" << cur << "file:" << nFileLen   <<std::endl;
  int iCount = 0;
  int iImuCount = 0;
  int iMultLaserCount = 0;
  int iPointCloudCount = 0;
  int64 time_stamp_imu =0;
  int64 time_stamp_laser =0; 
  clock_t first_laser = clock();
  clock_t first_imu = clock();
  clock_t start = clock();
  clock_t new_t = clock();
   ::ros::Time begin_time;
  std::deque<InstantMsg> delayed_messages;
  
  while( cur  <= nFileLen )//&& new_t -start <= FLAGS_duration*CLOCKS_PER_SEC*60)//&& iMultLaserCount < 199 && iPointCloudCount <199  )
  {
    new_t = clock();
    unsigned int flength = *(unsigned int*)((char*)mapped_mem+cur);
    cur += sizeof(unsigned int);
 
    int64 time_stamp = *(int64*)((char*)mapped_mem+cur);
    cur += sizeof(int64);
    unsigned char type = *(unsigned char*)((char*)mapped_mem+cur);
    cur += sizeof(unsigned char);
    if(cur + flength >= nFileLen )
    {
        std::cout << "====================== read file end =====================" << std::endl;
        break;
    }

    InstantMsg msg;
    int msg_len = msg.parsmsg(time_stamp,mapped_mem,type,cur,flength);
    if(iCount == 0)
       begin_time = msg.getTime();
     iCount++;
     
     
    while (!delayed_messages.empty()  &&  delayed_messages.front().getTime() < msg.getTime() + ::ros::Duration(1.)) 
    {
        const InstantMsg& delayed_msg = delayed_messages.front();
        const string topic = node.node_handle()->resolveName(delayed_msg.getTopic());//, false /* resolve */);
      //  const string topic = node.node_handle()->resolveName(delayed_msg.getTopic(), false /* resolve */);
         // ::ros::Time time = delayed_msg.getTime();
    
        if (delayed_msg.isType() == 4) {
          //   std::cout << ">james:poitcloud::iCount:"<< iCount << " timestamp:"  <<  delayed_msg.getTime() << " in64_time:"<< delayed_msg.time_stamp_ << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
             node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandlePointCloud2Message(
                  topic, delayed_msg.getPointCloud2());
        }
        if (delayed_msg.isType() == 1) {
         
//          std::cout << ">james:         Imu::iCount:"<< iCount << " timestamp:"  <<  delayed_msg.getTime() << " in64_time:"<< delayed_msg.time_stamp_ << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
          node.halo_imu_link_publisher_.publish(delayed_msg.getImu());
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleImuMessage(topic,delayed_msg.getImu());
         
        }
        delayed_messages.pop_front();
    }

    cur += msg_len;
  
    string topic ;
   topic = node.node_handle()->resolveName(msg.getTopic());//, false /* resolve */);
    //topic = node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
    
    if (expected_sensor_ids.count(topic) == 0) {
        continue;
    }
     //LOG_EVERY_N(INFO, 1000)<< "**[iCount] " << iCount << " sensor_id:" <<  msg.getTopic() << " topic:" << topic << "  expected_sensor_ids size:" << expected_sensor_ids.size() << "time_stamp:" << time_stamp << " begin_time:" << begin_time<<std::endl;
    
    delayed_messages.push_back(msg);
    //std::cout << "james >>>>>>>>>>>> Total:" << iCount << " ImuCout:" << iImuCount << " MultiLaserCount:" << iMultLaserCount << " PointCloudCount:" << iPointCloudCount << " delay:" << new_t - start << " duration:"<< CLOCKS_PER_SEC*60 <<std::endl;

    rosgraph_msgs::Clock clock;
    clock.clock = ToRos(time_stamp);
    clock_publisher.publish(clock);
    ::ros::spinOnce();
    new_t = ::clock();
   // LOG_EVERY_N(INFO, 100)
   //       << "Processed " << (msg.getTime() - begin_time).toSec()  << " bag time seconds... process time: " 
    //      << new_t-start  << " delayed_messages:" << delayed_messages.size() ;
    start = new_t;
  }

  std::cout << " ******************Total:" << iCount << " ImuCout:" << iImuCount << " MultiLaserCount:" << iMultLaserCount << " PointCloudCount:" << iPointCloudCount  << " duration:" << new_t - start << std::endl;
  fclose(fd);
  munmap(mapped_mem, nFileLen);
}



void Run(const std::string& bag_filenames) {

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = LoadOptions();

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  if (!FLAGS_urdf_filename.empty()) {
    urdf_transforms =
        ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  }
  tf_buffer.setUsingDedicatedThread(true);
//  uint32_t sec,nsec;
//  halo_time(sec, nsec);
  cartographer::common::Time time =TimeNow();// FromRos(sec,nsec);
  int64 seconds_since_epoch = std::chrono::system_clock::now().time_since_epoch().count();
  cartographer::common::Time time1 (
        ::cartographer::common::FromSeconds(
            seconds_since_epoch +
            ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds));//cartographer::common::Time::now();
  ::ros::Time ros_time = ToRos(time);
  //::ros::Time ros_time1 = ToRos((int64)clock());
  
  //std::cout << ">>>>>>>>>>>>>>>>>>>time:" <<time  << " time1:" << time1 << " time_stamp:" << ::cartographer::common::ToUniversal(time)  << " time1_stamp:" << ::cartographer::common::ToUniversal(time1) << " seconds_since_epoch:" << seconds_since_epoch << " ros_time:" << ros_time << " sizeof(clock_t):"<< sizeof(clock_t)<< std::endl;
 // ROS_INFO(">>>>>>>>>>>>>>>>james::halo_node_mian run file:%s FLAGS_slam_type:%d,FLAGS_use_bag_transforms:%d,halo_filename:%s\n",bag_filenames.c_str(),slam_type,FLAGS_use_bag_transforms,FLAGS_halo_filename.c_str());
  
  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;
  Node node(node_options, &tf_buffer);
  //int trajectory_id = node.StartTrajectoryWithDefaultTopics(trajectory_options);


  std::unordered_set<string> expected_sensor_ids;
  const auto check_insert = [&expected_sensor_ids, &node](const string& topic) {
   std::string name =  node.node_handle()->resolveName(topic);
    CHECK(expected_sensor_ids.insert(name)
              .second);
    ROS_INFO("expected_sensor_ids:,topic:%s , sensor_id:%s\n",name.c_str(),topic.c_str());
  };

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (trajectory_options.use_laser_scan) {
    check_insert(kLaserScanTopic);
  }
  if (trajectory_options.use_multi_echo_laser_scan) {
    check_insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  if (trajectory_options.num_point_clouds > 0) {
    for (int i = 0; i < trajectory_options.num_point_clouds; ++i) {
      // TODO(hrapp): This code is duplicated in places. Pull out a method.
      string topic = kPointCloud2Topic;
     // james
//      if (trajectory_options.num_point_clouds > 1) {
      if (trajectory_options.num_point_clouds >= 1) {
        topic += "_" + std::to_string(i + 1);
      }
      check_insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options.map_builder_options.use_trajectory_builder_3d() ||
      (node_options.map_builder_options.use_trajectory_builder_2d() &&
       trajectory_options.trajectory_builder_options
           .trajectory_builder_2d_options()
           .use_imu_data())) {
    check_insert(kImuTopic);
  }

  // For both 2D and 3D SLAM, odometry is optional.
  if (trajectory_options.use_odometry) {
    check_insert(kOdometryTopic);
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


  const string& bag_filename = bag_filenames;

  //for (const string& bag_filename : bag_filenames) 
  //do{
   // if (sigint_triggered) {
   //   break;
    //}

 const int trajectory_id = node.map_builder_bridge()->AddTrajectory(
        expected_sensor_ids, trajectory_options);
 
 if(slam_type == 1) 
  {  
     simulate_slam2(node,trajectory_id,bag_filename,clock_publisher,expected_sensor_ids);
  }else if (slam_type ==2)
  {
       /* code */
    simulate_imu_slam(node,trajectory_id,bag_filename,clock_publisher,expected_sensor_ids);
  }else{
    bool bRecord = false;
    if(!FLAGS_halo_filename.empty() && FLAGS_halo_filename != "none")
    {
      output.open(FLAGS_halo_filename.c_str(),std::ios::out|std::ios::trunc|std::ios::binary);
      std::string strImu = FLAGS_halo_filename + ".txt";
      imu_output.open(strImu.c_str(),std::ios::out|std::ios::trunc);
  //    ROS_INFO(">>>>>>>>>>>>>>>>james::open halo file:%s \n",FLAGS_halo_filename.c_str());
      bRecord = true;
    }
    int iCount = 0;
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const ::ros::Time begin_time = view.getBeginTime();
    const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();
   // ROS_INFO("<james::halo_node_mian run file:%s duration_in_seconds:%d>\n",bag_filename.c_str(),duration_in_seconds);
  
    // We make sure that tf_messages are published before any data messages, so
    // that tf lookups always work and that tf_buffer has a small cache size -
    // because it gets very inefficient with a large one.
    std::deque<rosbag::MessageInstance> delayed_messages;
    clock_t new_t = clock();
   clock_t start = clock();
 
    for (const rosbag::MessageInstance& msg : view) {
      if (sigint_triggered) {
        break;
      }
    // if(iCount == 50)
      //  break;

      iCount++;
        
      if (FLAGS_use_bag_transforms && msg.isType<tf2_msgs::TFMessage>()) {
       ROS_INFO("................................tf_publisher.publish(tf_message);.....................");
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
                 msg.getTime() + ::ros::Duration(1.)) {
        const rosbag::MessageInstance& delayed_msg = delayed_messages.front();
        const string topic = node.node_handle()->resolveName(
            delayed_msg.getTopic(), false /* resolve */);
         // ROS_INFO("......topic:%s,sensor_id:%s............................", topic.c_str(),delayed_msg.getTopic().c_str());
       
       
        if (delayed_msg.isType<sensor_msgs::LaserScan>()) {

          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleLaserScanMessage(
                  topic, delayed_msg.instantiate<sensor_msgs::LaserScan>());
//          ROS_INFO(">james:LaserScan::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
 
        }
        if (delayed_msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleMultiEchoLaserScanMessage(
                  topic,
                  delayed_msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
  //        ROS_INFO(">james:MultiEchoLaserScan::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
        }
        if (delayed_msg.isType<sensor_msgs::PointCloud2>()) 
        {
          boost::shared_ptr<sensor_msgs::PointCloud2> point_cloud_msg = delayed_msg.instantiate<sensor_msgs::PointCloud2>();
            if(bRecord)
            {
           const cartographer::common::Time time = FromRos(point_cloud_msg->header.stamp);
             int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
            std::cout << ">james:poitcloud::iCount:"<< iCount << " timestamp:"  <<  point_cloud_msg->header.stamp << " in64_time:"<< uts_timestamp << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
              write_halo_file(output,delayed_msg.getTopic(),point_cloud_msg);

        //      LOG_EVERY_N(INFO, 1) << ">james:PointCloud2::iCount:"<< iCount << " timestamp:"  <<  point_cloud_msg->header.stamp <<" topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic();
  //      ROS_INFO(">james:PointCloud2::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
            }
              node.map_builder_bridge()
            ->sensor_bridge(trajectory_id)
            ->HandlePointCloud2Message(topic, point_cloud_msg);
          
        }
        if (delayed_msg.isType<sensor_msgs::Imu>()) {
          boost::shared_ptr<sensor_msgs::Imu> imu_msg = delayed_msg.instantiate<sensor_msgs::Imu>();
          if(bRecord)
          { 
             write_halo_file(output,imu_msg);
             const cartographer::common::Time time = FromRos(imu_msg->header.stamp);
             int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
//            std::cout << ">james:         Imu::iCount:"<< iCount << " timestamp:"  <<  imu_msg->header.stamp << " in64_time:"<< uts_timestamp << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
          }
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleImuMessage(topic,imu_msg);
             
      //    ROS_INFO(">james:Imu::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
        }

        if (delayed_msg.isType<nav_msgs::Odometry>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleOdometryMessage(
                  topic, delayed_msg.instantiate<nav_msgs::Odometry>());
        //  ROS_INFO(">james:Odometry::topic:%s,delayed_msg.getTopic:%s<\n",topic.c_str(), delayed_msg.getTopic().c_str());
                }
        delayed_messages.pop_front();
      }

     const string topic =
      node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
      
      if (expected_sensor_ids.count(topic) == 0) {
        continue;
      }
     
      delayed_messages.push_back(msg);

      rosgraph_msgs::Clock clock;
      clock.clock = msg.getTime();
      clock_publisher.publish(clock);

      ::ros::spinOnce();
      new_t = ::clock();
    //  LOG_EVERY_N(INFO, 100000)
    //  LOG_EVERY_N(INFO, 100)
    //      << "Processed " << (msg.getTime() - begin_time).toSec() << " clock time: " 
    //      << new_t-start  << " delayed_messages:" << delayed_messages.size() ;
    start = new_t;
    }
//james
    bag.close();

    if(bRecord)
    {
      output.close();
      imu_output.close();
       ROS_INFO("................................james:close halo file.....................");
       
    }
  }//while(0);

   ROS_INFO(">>>>>>>>>>>>>END SLAM write Assets %s <<<<<<<<<<<<<\n",bag_filenames.c_str());
     
  node.map_builder_bridge()->FinishTrajectory(trajectory_id);
  node.map_builder_bridge()->SerializeState(bag_filenames);
  node.map_builder_bridge()->WriteAssets(bag_filenames);
//  node.map_builder_bridge()->SerializeState(bag_filenames.front());
//  node.map_builder_bridge()->WriteAssets(bag_filenames.front());
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
   CHECK(!FLAGS_slam_type.empty())
      << "-slam_type is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  if(FLAGS_slam_type == "1")
    cartographer_ros::slam_type =1;
  else if (FLAGS_slam_type == "2")
  {
      cartographer_ros::slam_type =2;
  }else
    cartographer_ros::slam_type = 0;
  std::signal(SIGINT, &::cartographer_ros::SigintHandler);
  //::ros::init(argc, argv, "cartographer_offline_node",
   //           ::ros::init_options::NoSigintHandler);
  ::ros::init(argc, argv, "cartographer_node");
  
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run(FLAGS_bag_filenames);

  ::ros::shutdown();
}
