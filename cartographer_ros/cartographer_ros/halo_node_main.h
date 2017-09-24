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

#ifndef CARTOGRAPHER_ROS_HALO_NODE_MAIN_
#define CARTOGRAPHER_ROS_HALO_NODE_MAIN_
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
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/split_string.h"
#include "cartographer_ros/urdf_reader.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"
namespace cartographer_ros {
namespace {

std::fstream output;
std::fstream imu_output;

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
      cartographer::sensor::proto::CompressedPointCloud pts;
      pts.ParseFromArray(((char*)base_+offset),length_);
      cartographer::sensor::PointCloud ranges = cartographer::sensor::CompressedPointCloud(pts).Decompress();
      std::string frame_id ="" ;
      std::string sensor_id = cSensor_id;
   
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
      cartographer::sensor::proto::ImuData imuData;
      memcpy(&imuData,(void*)((char*)base_+offset_),length_);
     cartographer::sensor::ImuData imu = cartographer::sensor::FromProto(imuData);
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

void write_halo_file( std::fstream& output,const std::string& sensor_id, const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(*msg, pcl_point_cloud);
    cartographer::sensor::PointCloud point_cloud;
    for (const auto& point : pcl_point_cloud) {
      point_cloud.emplace_back(point.x, point.y, point.z);
    }
    unsigned char type = 4;
    cartographer::sensor::CompressedPointCloud pts(point_cloud);
    unsigned int len = pts.ToProto().ByteSize();
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
    pts.ToProto().SerializeToOstream(&output);
}

void write_halo_file( std::fstream& output, const sensor_msgs::Imu::ConstPtr& msg)
{
   //james
    cartographer::sensor::ImuData imu;
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
    cartographer::sensor::proto::ImuData protoImu = cartographer::sensor::ToProto(imu);
    unsigned int len = sizeof(protoImu);
    output.write((char*)&len,sizeof(unsigned int));
    output.write((char*)&uts_timestamp,sizeof(int64));
    unsigned char type = 1;  
    output.write((char*)&type,sizeof(unsigned char));
    output.write((char*)&protoImu,len);
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
       node.HandleImuMessage(trajectory_id, topic,msg);
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
         
              node.HandlePointCloud2Message(trajectory_id, topic,delayed_msg.getPointCloud2());
        }
        if (delayed_msg.isType() == 1) {
         
//          std::cout << ">james:         Imu::iCount:"<< iCount << " timestamp:"  <<  delayed_msg.getTime() << " in64_time:"<< delayed_msg.time_stamp_ << " topic:" << topic <<" delayed_msg.getTopic:" << delayed_msg.getTopic() << std::endl;
         // node.halo_imu_link_publisher_.publish(delayed_msg.getImu());
          node.HandleImuMessage(trajectory_id, topic,delayed_msg.getImu());         
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

}  // namespace
}  // namespace cartographer_ros

#endif
