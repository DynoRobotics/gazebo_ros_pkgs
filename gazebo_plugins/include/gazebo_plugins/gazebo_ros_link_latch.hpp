#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_LINK_LATCH_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_LINK_LATCH_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosLinkLatchPrivate;

/// Link Latch plugin for attracting entities around the model like a trailer to a car
/*
 * \author  Christoffer Johannesson (based on Vacuum Gripper by Kentaro Wada)
 *
 * \date 9 Feb 2024
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name='link_latch' filename='libgazebo_ros_link_latch.so'>

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remapping service and topic names -->
        <remapping>switch:=custom_switch</remapping>
        <remapping>grasping:=custom_grasping</remapping>
      </ros>

      <!-- Link associated with gripper -->
      <link_name>link</link_name>

      <!-- Max distance to attract entities -->
      <max_distance>10.0</max_distance>

      <!-- List of entities that can be latched to-->
      <latchable>wastebin1</latchable>
      <latchable>wastebin2</latchable>

    </plugin>
  \endcode
*/
class GazeboRosLinkLatch : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosLinkLatch();

  /// Destructor
  ~GazeboRosLinkLatch();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  /// Help functions
  void AttatchLink();
  void DetachLink();

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosLinkLatchPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_LINK_LATCH_HPP_
