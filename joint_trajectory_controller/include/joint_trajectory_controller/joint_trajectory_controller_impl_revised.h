///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser

#ifndef JOINT_TRAJECTORY_CONTROLLER_IMPL_REVISED
#define JOINT_TRAJECTORY_CONTROLLER_IMPL_REVISED

#include <string>
#include <vector>

// Boost
#include <boost/shared_ptr.hpp>

// URDF
#include <urdf/model.h>

namespace joint_trajectory_controller_revised
{

namespace internal
{

template <class Enclosure, class Member>
inline boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    XmlRpc::XmlRpcValue& elem = xml_array[i];
    if (elem.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(elem));
  }
  return out;
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  urdf::ModelSharedPtr urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str))
  {
    if (!urdf->initString(urdf_str))
    {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
        nh.getNamespace() << ").");
      return urdf::ModelSharedPtr();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description"))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
    return urdf::ModelSharedPtr();
  }
  return urdf;
}

std::vector<urdf::JointConstSharedPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
{
  std::vector<urdf::JointConstSharedPtr> out;
  for (const auto& joint_name : joint_names)
  {
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in URDF model.");
      return std::vector<urdf::JointConstSharedPtr>();
    }
  }
  return out;
}

std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  pure_joint_trajectory_controller::EditableControllerSettings controller_settings;

  using namespace internal;
  controller_settings.setControllerName( getLeafNamespace(controller_nh) );

  double state_publish_rate = 50.0;
  controller_nh_.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED(controller_settings.getControllerName(),
                         "Controller state will be published at " << state_publish_rate << "Hz.");
  controller_settings.setStatePublishRate(ros::Duration(1.0 / state_publish_rate));

  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  controller_settings.setActionMonitorRate(ros::Duration(1.0 / action_monitor_rate));
  ROS_DEBUG_STREAM_NAMED(controller_settings.getControllerName(),
                         "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  double stop_trajectory_duration = 0.0;
  controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration);
  ROS_DEBUG_STREAM_NAMED(controller_settings.getControllerName(),
                         "Stop trajectory has a duration of " << stop_trajectory_duration << "s.");
  controller_settings.setStopTrajectoryDuration(stop_trajectory_duration);

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{

}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time)
{

}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{

}

}

#endif // JOINT_TRAJECTORY_CONTROLLER_IMPL_REVISED
