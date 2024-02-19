// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * \brief Link Latch plugin for attracting entities around the model like a trailer to a car
 *
 * \author Christoffer Johannesson (based on Vacuum Gripper by Kentaro Wada)
 *
 * \date 9 Feb 2024
 */

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_link_latch.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sdf/sdf.hh>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

namespace gazebo_plugins
{
class GazeboRosLinkLatchPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  void OnUpdate();

  /// \brief Function to switch the latch on/off.
  /// \param[in] req Request
  /// \param[out] res Response
  void OnSwitch(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  /// Help function to attach link
  gazebo::physics::LinkPtr GetLink();
  void AttachLink(gazebo::physics::LinkPtr &link, ignition::math::Pose3d &diff);
  void DetachLink();

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publisher for latch action status
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

  /// Service for latch switch
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;


  /// Pointer to latch link.
  gazebo::physics::LinkPtr latch_link_;

  /// Pointer to latch joint
  gazebo::physics::JointPtr latched_joint_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// True if latch is on.
  bool status_;

  /// Entities that can be latched to.
  std::unordered_set<std::string> latchable_;

  /// Max distance to apply force.
  double max_distance_;
};

GazeboRosLinkLatch::GazeboRosLinkLatch()
: impl_(std::make_unique<GazeboRosLinkLatchPrivate>())
{

}

GazeboRosLinkLatch::~GazeboRosLinkLatch()
{
}

void GazeboRosLinkLatch::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{ 
  impl_->model_ = _model;
  impl_->world_ = _model->GetWorld();
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (_sdf->HasElement("latch_link")) {
    auto link = _sdf->Get<std::string>("latch_link");
    impl_->latch_link_ = _model->GetLink(link);
    if (!impl_->latch_link_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Link [%s] not found. Aborting", link.c_str());
      impl_->ros_node_.reset();
      return;
    }
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Please specify <latch_link>. Aborting.");
  }

  impl_->max_distance_ = _sdf->Get<double>("max_distance", 0.05).first;

  if (_sdf->HasElement("latchable")) {
    for (auto latchable = _sdf->GetElement("latchable"); latchable != nullptr;
      latchable = latchable->GetNextElement("latchable"))
    {
      auto name = latchable->Get<std::string>();
      impl_->latchable_.insert(name);
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Model/Link [%s] is latchable for the robot", name.c_str());
    }
  }

  // Initialize publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>(
    "grasping", qos.get_publisher_qos("grasping", rclcpp::QoS(1)));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise latch status on [%s]", impl_->pub_->get_topic_name());

  // Initialize service
  impl_->service_ = impl_->ros_node_->create_service<std_srvs::srv::SetBool>(
    "switch",
    std::bind(
      &GazeboRosLinkLatchPrivate::OnSwitch, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise latch switch service on [%s]", impl_->service_->get_service_name());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosLinkLatchPrivate::OnUpdate, impl_.get()));
}

void GazeboRosLinkLatchPrivate::OnUpdate()
{
  std_msgs::msg::Bool latching_msg;
  if (!status_) {
    pub_->publish(latching_msg);
    return;
  }

  // Check if the robot is already latch to a link
  if (latched_joint_ != nullptr) {
    // TODO: Add some physic check listening to a topic with joint force from a ft_sensor
    bool break_latch = false;
    if(break_latch)
    {
      DetachLink();
    }
  }

  latching_msg.data = true;
  pub_->publish(latching_msg);
}

gazebo::physics::LinkPtr GazeboRosLinkLatchPrivate::GetLink(){
  std::lock_guard<std::mutex> lock(lock_);

  ignition::math::Pose3d parent_pose = latch_link_->WorldPose();
  gazebo::physics::Model_V models = world_->Models();

  // Check all latchable objects
  for (auto & model : models) {
    if (latchable_.find(model->GetName()) == latchable_.end()) {
      continue;
    }
    gazebo::physics::Link_V links = model->GetLinks();
    for (auto & link : links) {
      ignition::math::Pose3d link_pose = link->WorldPose();
      ignition::math::Pose3d diff = parent_pose - link_pose;
      if (diff.Pos().Length() > max_distance_) {
        continue;
      }
      return link;
    }
  }
  return nullptr;
}

void GazeboRosLinkLatchPrivate::AttachLink(gazebo::physics::LinkPtr &link, ignition::math::Pose3d &diff)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Model/Link [%s] is attaching to robot", link->GetName().c_str());

  latched_joint_ = model_->CreateJoint("latch_dynamic_joint", "fixed", latch_link_, link);
  latched_joint_->Init();
  status_ = true;
}

void GazeboRosLinkLatchPrivate::DetachLink()
{
  latched_joint_->Detach();
  model_->RemoveJoint(latched_joint_->GetName());
  latched_joint_ = nullptr;
  status_ = false;
}

void GazeboRosLinkLatchPrivate::OnSwitch(
  std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  res->success = false;
  if (req->data) {
    if (!status_) {
      auto link = GetLink();
      if (link == nullptr) {
        RCLCPP_WARN(ros_node_->get_logger(), "No latchable link found");
        return;
      }
      auto pose = ignition::math::Pose3d();
      AttachLink(link, pose);
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Latch is already on");
    }
  } else {
    if (status_) {
      DetachLink();
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Latch is already off");
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosLinkLatch)
}  // namespace gazebo_plugins
