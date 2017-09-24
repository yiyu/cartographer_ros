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

#include "cartographer_ros/time_conversion.h"

#include "cartographer/common/time.h"
#include "ros/ros.h"

namespace cartographer_ros {

::ros::Time ToRos(::cartographer::common::Time time) {
  int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
  int64 ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  ::ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
::cartographer::common::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}


::ros::Time ToRos(int64 uts_timestamp ) {
  int64 ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  // std::cout << "====================== test getTime 0 uts_timestamp=====================" << uts_timestamp << " ns_since_unix_epoch:" <<ns_since_unix_epoch<< std::endl;
 //if(ns_since_unix_epoch < 0)
  //  ns_since_unix_epoch = uts_timestamp;
  ::ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
 // std::cout << "====================== test getTime 0 uts_timestamp=====================" << uts_timestamp << " ros_time:" <<ros_time<< std::endl;
  return ros_time;
}

::cartographer::common::Time FromRos(uint32_t& sec, uint32_t& nsec){
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (nsec + 50) / 100);  // + 50 to get the rounding correct.
}

int64  NowTimestamp(){
  uint32_t sec,nsec;
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
 return  (sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (nsec + 50) / 100; 
}

::cartographer::common::Time TimeNow(){

  return ::cartographer::common::FromUniversal(NowTimestamp());
}

}  // namespace cartographer_ros
