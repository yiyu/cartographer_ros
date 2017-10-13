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

#include "cartographer_ros/map_builder_bridge.h"

#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/msg_conversion.h"


#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace carto = ::cartographer;
/*
void WriteTrajectory(const std::vector<::cartographer::mapping::TrajectoryNode>&
                         trajectory_nodes,
                     const std::string& stem) {
  carto::mapping::proto::Trajectory trajectory;
    

  // TODO(whess): Add multi-trajectory support.
  for (const auto& node : trajectory_nodes) {
    const auto& data = *node.constant_data;
    auto* node_proto = trajectory.add_node();
    node_proto->set_timestamp(carto::common::ToUniversal(data.time));
    *node_proto->mutable_pose() =
        carto::transform::ToProto(node.pose );//james * data.tracking_to_pose);
  }

  // Write the trajectory.
  std::ofstream proto_file(stem + ".pb",
                           std::ios_base::out | std::ios_base::binary);
  CHECK(trajectory.SerializeToOstream(&proto_file))
      << "Could not serialize trajectory.";
  proto_file.close();
  CHECK(proto_file) << "Could not write trajectory.";
}
*/




namespace cartographer_ros {

namespace {

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options,
                                   tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(node_options.map_builder_options),
      tf_buffer_(tf_buffer) {}

void MapBuilderBridge::LoadMap(const std::string& map_filename) {
  LOG(INFO) << "Loading map '" << map_filename << "'...";
  cartographer::io::ProtoStreamReader stream(map_filename);
  map_builder_.LoadMap(&stream);
}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_.AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.num_subdivisions_per_laser_scan,
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_.FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
}

void MapBuilderBridge::SerializeState(const std::string& filename) {
  std::cout << "SerializeState filename:" << filename << std::endl;
  cartographer::io::ProtoStreamWriter writer(filename);
  map_builder_.SerializeState(&writer);
  CHECK(writer.Close()) << "Could not write state.";

  Write3DAssets(filename);
/*
  cartographer::io::ProtoStreamReader stream(filename);
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  stream.ReadProto(&pose_graph_proto);
  std::cout << "SerializeState:Pose graphs contains " << pose_graph_proto.trajectory_size() << std::endl;
  std::vector<::cartographer::mapping::proto::Trajectory> all_trajectories(
      pose_graph_proto.trajectory().begin(),
      pose_graph_proto.trajectory().end());

 double voxel_size = trajectory_options_[0]
              .trajectory_builder_options.trajectory_builder_3d_options()
              .submaps_options()
              .high_resolution();
  std::cout << "SerializeState:Writing 3D assets with voxel_size:"<<voxel_size <<" stem '" << filename << "'..." << std::endl;
    
  Write3DAssets(all_trajectories,voxel_size,filename);*/
}

void MapBuilderBridge::Write3DAssets(const std::string& stem)
{
  double voxel_size = trajectory_options_[0]
              .trajectory_builder_options.trajectory_builder_3d_options()
              .submaps_options()
              .high_resolution();
  carto::io::XRayPointsProcessor::DrawTrajectories draw = carto::io::XRayPointsProcessor::DrawTrajectories::kYes;
  carto::mapping::proto::SparsePoseGraph pose_graph_proto = map_builder_.sparse_pose_graph()->ToProto();;
  
  std::vector<::cartographer::mapping::proto::Trajectory> all_trajectories(
      pose_graph_proto.trajectory().begin(),
      pose_graph_proto.trajectory().end());

  std::cout << "SerializeState:Writing 3D assets with voxel_size:"<<voxel_size <<" stem '" << stem << "'..." << std::endl;
  const auto file_writer_factory = [](const string& stem) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(stem);
  };

  carto::io::NullPointsProcessor null_points_processor;
  carto::io::XRayPointsProcessor xy_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitY())),
      {}, draw,stem + "_xray_xy", all_trajectories,file_writer_factory, &null_points_processor);
  std::cout << "Write3DAssets: file:" << stem + "_xray_xy" << std::endl;
  carto::io::XRayPointsProcessor yz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())),
      {}, draw,stem + "_xray_yz",all_trajectories, file_writer_factory, &xy_xray_points_processor);
  
  carto::io::XRayPointsProcessor xz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitZ())),
      {},draw, stem + "_xray_xz", all_trajectories,file_writer_factory, &yz_xray_points_processor);
  
  carto::io::PlyWritingPointsProcessor ply_writing_points_processor(
      file_writer_factory(stem + ".ply"), &xz_xray_points_processor);

  const auto all_trajectory_nodes = map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
   std::cout << "all_trajectory_nodes.size():" << all_trajectory_nodes.size() << "" << std::endl;
  
  for (int trajectory_id = 0;trajectory_id < static_cast<int>(all_trajectory_nodes.size());++trajectory_id) 
  {
    const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
    //marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
     std::cout << "....single_trajectory_nodes.size():" << single_trajectory_nodes.size() << "" << std::endl;
    for (const auto& node : single_trajectory_nodes) 
    {
      if (node.trimmed())
        continue;
      auto points_batch = carto::common::make_unique<carto::io::PointsBatch>(); 
      points_batch->origin = node.pose.translation().cast<float>();
      const carto::sensor::PointCloud& point_cloud = node.constant_data->high_resolution_point_cloud;
       std::cout << "........point_cloud.size():" << point_cloud.size() << "" << std::endl;
  
      for (size_t i = 0; i < point_cloud.size(); ++i) 
      {
        points_batch->points.push_back(point_cloud[i]);
      }
     // xy_xray_points_processor.Process(std::move(points_batch));  
      ply_writing_points_processor.Process(std::move(points_batch));

    }
    std::cout << "trajectory_id:"<< trajectory_id <<std::endl;
  }

  ply_writing_points_processor.Flush();
  
 // xz_xray_points_processor.Flush();
   // xy_xray_points_processor.Flush();
     std::cout << "ply_writing_points_processor.Flush():"<< std::endl;
}

bool MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  const std::string error =
      map_builder_.SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    return false;
  }

  response.submap_version = response_proto.submap_version();
  CHECK(response_proto.textures_size() > 0)
      << "empty textures given for submap: " << submap_id;

  // TODO(gaschler): Forward all textures, not just the first one.
  const auto& texture_proto = *response_proto.textures().begin();
  response.cells.insert(response.cells.begin(), texture_proto.cells().begin(),
                        texture_proto.cells().end());
  response.width = texture_proto.width();
  response.height = texture_proto.height();
  response.resolution = texture_proto.resolution();
  response.slice_pose = ToGeometryMsgPose(
      cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  return true;
}

cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  for (size_t trajectory_id = 0; trajectory_id < all_submap_data.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index < all_submap_data[trajectory_id].size(); ++submap_index) {
      const auto& submap_data = all_submap_data[trajectory_id][submap_index];
      if (submap_data.submap == nullptr) {
        continue;
      }
      cartographer_ros_msgs::SubmapEntry submap_entry;
      submap_entry.trajectory_id = trajectory_id;
      submap_entry.submap_index = submap_index;
      submap_entry.submap_version = submap_data.submap->num_range_data();
      submap_entry.pose = ToGeometryMsgPose(submap_data.pose);
      submap_list.submap.push_back(submap_entry);
    }
  }
  return submap_list;
}

std::unordered_map<int, MapBuilderBridge::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    const cartographer::mapping::TrajectoryBuilder* const trajectory_builder =
        map_builder_.GetTrajectoryBuilder(trajectory_id);
    const cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate =
        trajectory_builder->pose_estimate();
    if (cartographer::common::ToUniversal(pose_estimate.time) < 0) {
      continue;
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    trajectory_states[trajectory_id] = {
        pose_estimate,
        map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(
            trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            pose_estimate.time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return trajectory_states;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  for (int trajectory_id = 0;
       trajectory_id < static_cast<int>(all_trajectory_nodes.size());
       ++trajectory_id) {
    const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
    visualization_msgs::Marker marker;
    marker.ns = "Trajectory " + std::to_string(trajectory_id);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.stamp = ::ros::Time::now();
    marker.header.frame_id = node_options_.map_frame;
    marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
    marker.scale.x = kTrajectoryLineStripMarkerScale;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.z = 0.05;
    for (const auto& node : single_trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node.pose.translation());
      marker.points.push_back(node_point);
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
        marker.points.clear();
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    trajectory_node_list.markers.push_back(marker);
  }
  return trajectory_node_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_marker = constraint_intra_marker;
  constraint_inter_marker.id = marker_id++;
  constraint_inter_marker.ns = "Inter constraints";
  constraint_inter_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_marker = constraint_intra_marker;
  residual_inter_marker.id = marker_id++;
  residual_inter_marker.ns = "Inter residuals";
  residual_inter_marker.pose.position.z = 0.1;

  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  const auto constraints = map_builder_.sparse_pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::SparsePoseGraph::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      constraint_marker = &constraint_inter_marker;
      residual_marker = &residual_inter_marker;
      // Bright yellow
      color_constraint.a = 1.0;
      color_constraint.r = color_constraint.g = 1.0;
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto& submap_data =
        all_submap_data[constraint.submap_id.trajectory_id]
                       [constraint.submap_id.submap_index];
    const auto& submap_pose = submap_data.pose;
    const auto& trajectory_node_pose =
        all_trajectory_nodes[constraint.node_id.trajectory_id]
                            [constraint.node_id.node_index]
                                .pose;
    const cartographer::transform::Rigid3d constraint_pose =
        submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_marker);
  constraint_list.markers.push_back(residual_inter_marker);
  return constraint_list;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

//james ........
visualization_msgs::MarkerArray MapBuilderBridge::GetHaloTrajectoryNodesList() {
  visualization_msgs::MarkerArray trajectory_nodes_list;
   cartographer::mapping::TrajectoryBuilder*  trajectory_builder = map_builder_.GetTrajectoryBuilder(0);
  auto trajectory_nodes = trajectory_builder->GetHaloTrajectoryNodes();
  int marker_id = 0;
  //std::cout <<  "<<<james::MapBuilderBridge::GetHaloTrajectoryNodesList:>>>trajectory_nodes size:";
    visualization_msgs::Marker marker;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.stamp = ::ros::Time::now();
    marker.header.frame_id = node_options_.map_frame;
    marker.color.b = 1.0;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = kTrajectoryLineStripMarkerScale;
    marker.pose.orientation.w = 1.0;
    for (const auto& node : trajectory_nodes) {
      marker.points.push_back(ToGeometryMsgPoint(
          (node * cartographer::transform::Rigid3d::Identity()).translation()));
     // std::cout << " pose:"<<  node.pose  << " tracking_to_pose:"<< node.constant_data->tracking_to_pose;
    }
    trajectory_nodes_list.markers.push_back(marker);
  
 /* LOG(INFO) <<  " trajectory_nodes size:" << trajectory_nodes.size() << 
                " trajectory_nodes_list:"<< trajectory_nodes_list.markers.size() <<
                " points.size:" << trajectory_nodes_list.markers[0].points.size() << std::endl ;
 */
  return trajectory_nodes_list;
}

::geometry_msgs::PoseArray MapBuilderBridge::GetHaloPoseList() 
{
  ::geometry_msgs::PoseArray halo_pose_list;
  cartographer::mapping::TrajectoryBuilder*  trajectory_builder = map_builder_.GetTrajectoryBuilder(0);
  auto trajectory_nodes = trajectory_builder->GetHaloTrajectoryNodes();
  int marker_id = 0; 
  halo_pose_list.header.seq = marker_id++;
  halo_pose_list.header.stamp =  ::ros::Time::now();
  halo_pose_list.header.frame_id = node_options_.map_frame;
  for (const auto& node : trajectory_nodes) {
   // marker.points.push_back(ToGeometryMsgPoint(
    //    (node * cartographer::transform::Rigid3d::Identity()).translation()));
      geometry_msgs::Pose pose;
      pose.position = ToGeometryMsgPoint(node.translation());
      pose.orientation.w = node.rotation().w();
      pose.orientation.x = node.rotation().x();
      pose.orientation.y = node.rotation().y();
      pose.orientation.z = node.rotation().z();
      halo_pose_list.poses.push_back(pose);
  }
   
  
 /* LOG(INFO) <<  " trajectory_nodes size:" << trajectory_nodes.size() << 
                " trajectory_nodes_list:"<< trajectory_nodes_list.markers.size() <<
                " points.size:" << trajectory_nodes_list.markers[0].points.size() << std::endl ;
 */
  return halo_pose_list;
}

::geometry_msgs::PoseArray MapBuilderBridge::GetHaloImuList() 
{
  ::geometry_msgs::PoseArray halo_pose_list;
  cartographer::mapping::TrajectoryBuilder*  trajectory_builder = map_builder_.GetTrajectoryBuilder(0);
  auto trajectory_nodes = map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  int marker_id = 0; 
  halo_pose_list.header.seq = marker_id++;
  halo_pose_list.header.stamp =  ::ros::Time::now();
  halo_pose_list.header.frame_id = node_options_.map_frame;

  for (const auto& single_trajectory : trajectory_nodes) {
    for (const auto& node : single_trajectory) {
      ::cartographer::transform::Rigid3d rigid3d = node.pose;// (node.pose * node.constant_data->tracking_to_pose);
      geometry_msgs::Pose pose;
      pose.position = ToGeometryMsgPoint(rigid3d.translation());
      pose.orientation.w = rigid3d.rotation().w();
      pose.orientation.x = rigid3d.rotation().x();
      pose.orientation.y = rigid3d.rotation().y();
      pose.orientation.z = rigid3d.rotation().z();
      halo_pose_list.poses.push_back(pose);
    }
  }
  return halo_pose_list;
}
//////
}  // namespace cartographer_ros
