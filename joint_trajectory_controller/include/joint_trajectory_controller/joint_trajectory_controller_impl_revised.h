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

// Project
#include <joint_trajectory_controller/helper_functions.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

namespace joint_trajectory_controller_revised
{

static ros::Duration getStatePublishRate(ros::NodeHandle& controller_nh, const std::string& controller_name)
{
  double state_publish_rate = 50.0;
  controller_nh.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED(controller_name, "Controller state will be published at " << state_publish_rate << "Hz.");
  return ros::Duration(1.0 / state_publish_rate);
}

static ros::Duration getActionMonitorRate(ros::NodeHandle& controller_nh, const std::string& controller_name)
{
  double action_monitor_rate = 20.0;
  controller_nh.getParam("action_monitor_rate", action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(controller_name,
                         "Action status changes will be monitored at " << action_monitor_rate << "Hz.");
  return ros::Duration(1.0 / action_monitor_rate);
}

static double getStopTrajectoryDuration(ros::NodeHandle& controller_nh, const std::string& controller_name)
{
  double stop_trajectory_duration = 0.0;
  controller_nh.getParam("stop_trajectory_duration", stop_trajectory_duration);
  ROS_DEBUG_STREAM_NAMED(controller_name, "Stop trajectory has a duration of " << stop_trajectory_duration << "s.");
  return stop_trajectory_duration;
}

static bool arePartialJointGoalsAllowed(ros::NodeHandle& controller_nh, const std::string& controller_name)
{
  bool allow_partial_joints_goal {false} ;
  controller_nh.param<bool>("allow_partial_joints_goal", allow_partial_joints_goal, false);
  if (allow_partial_joints_goal)
  {
    ROS_DEBUG_NAMED(controller_name, "Goals with partial set of joints are allowed");
  }
  return allow_partial_joints_goal;
}

template<class HardwareInterface, class JointHandle>
static std::vector<JointHandle> getJointHandles(HardwareInterface* hw,
                                                const std::vector<std::string>& joint_names,
                                                const std::string& controller_name)
{
  std::vector<JointHandle> joint_handles;
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    try
    {
      joint_handles.emplace_back( hw->getHandle(joint_names.at(i)) );
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(controller_name, "Could not find joint handle for joint '" << joint_names.at(i) << "'");
      joint_handles.clear();
      return joint_handles;
    }
  }
  return joint_handles;
}

template <class SegmentImpl, class HardwareInterface>
JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
JointTrajectoryControllerRevised()
  : controller_interface::Controller<HardwareInterface>()
{

}

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh; // TODO: Is this copy really necessary?

  ros::NodeHandle tol_nh(controller_nh, "constraints");

  using namespace joint_trajectory_controller;
  using namespace joint_trajectory_controller::internal;

  const std::string controller_name {getLeafNamespace(controller_nh)};
  controller_name_ = controller_name;
  const std::vector<std::string> joint_names {getStrings(controller_nh, "joints")};
  if (joint_names.empty())
  {
    ROS_ERROR_STREAM_NAMED(controller_name, "No joint names found");
    return false;
  }
  const long unsigned int num_joints {joint_names.size()};

  using Scalar = typename Segment::Scalar;

  EditableControllerSettings<joint_trajectory_controller::SegmentTolerances<Scalar> > controller_settings;
  controller_settings.setControllerName(controller_name);
  controller_settings.setStatePublishRate(getStatePublishRate(controller_nh, controller_name));
  controller_settings.setActionMonitorRate(getActionMonitorRate(controller_nh, controller_name));
  controller_settings.setStopTrajectoryDuration(getStopTrajectoryDuration(controller_nh, controller_name));
  controller_settings.setDefaultSegmentTolerances(getSegmentTolerances<Scalar>(tol_nh, joint_names));
  controller_settings.setPartialJointGoals(arePartialJointGoalsAllowed(controller_nh, controller_name));


  std::unique_ptr<EditableHWInputHandleSettings> input_settings {new EditableHWInputHandleSettings()};

  input_settings->setJointNames(joint_names);
  try
  {
    input_settings->setAngleWrapAround( getAngleWraparound(root_nh, joint_names) );
  }
  catch (...)
  {
    return false;
  }

  joint_handles_ = getJointHandles<HardwareInterface, JointHandle>(hw, joint_names, controller_name);
  if (joint_handles_.size() != joint_names.size())
  {
    ROS_ERROR_STREAM_NAMED(controller_name, "Incorrect number of joint handles");
  }

  using InputHandle = JointHandleHWInputHandle<typename Segment::State, typename HardwareInterface::ResourceHandleType>;
  InputHandle input_handle(std::move(input_settings), joint_handles_);

  using HwIfaceAdapter = HardwareInterfaceAdapter<HardwareInterface, typename Segment::State>;
  using OutputHandle = HWOutputHandleForHWInterfaceAdapter<typename Segment::State, HwIfaceAdapter>;

  std::unique_ptr<HwIfaceAdapter> hw_adapter {new HwIfaceAdapter()};
  hw_adapter->init(joint_handles_, controller_nh);
  OutputHandle output_handle(std::move(hw_adapter));

  setupROSConnections(controller_nh);

  ROS_DEBUG_STREAM_NAMED(controller_name,
                         "Initialized controller '" << controller_name << "' with:" <<
                         "\n- Number of joints: " << num_joints <<
                         "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
                         "\n- Trajectory segment type: '"
                         << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");
  return true;
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
setupROSConnections(ros::NodeHandle& controller_nh)
{
  trajectory_command_sub_ = controller_nh.subscribe("command", 1, &JointTrajectoryControllerRevised::trajectoryCommandCB, this);
  state_publisher_.reset(new StatePublisher(controller_nh, "state", 1));
  action_server_.reset(new ActionServer(controller_nh, "follow_joint_trajectory",
                                        boost::bind(&JointTrajectoryControllerRevised::goalCB,   this, _1),
                                        boost::bind(&JointTrajectoryControllerRevised::cancelCB, this, _1),
                                        false));
  action_server_->start();
  query_state_service_ = controller_nh.advertiseService("query_state",
                                                        &JointTrajectoryControllerRevised::queryStateService,
                                                        this);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{
  pure_controller_->starting(time);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time)
{
  pure_controller_->stopping(time);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  pure_controller_->update(time, period);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(controller_name_,"Received new action goal");

  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(controller_name_, "Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; // TODO: Add better error status to msg?
    gh.setRejected(result);
    return;
  }

  pure_controller_->goalCB(gh);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
cancelCB(GoalHandle gh)
{
  pure_controller_->cancelCB(gh);
}

template <class SegmentImpl, class HardwareInterface>
inline bool JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                  control_msgs::QueryTrajectoryState::Response& resp)
{
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(controller_name_, "Can't sample trajectory. Controller is not running.");
    return false;
  }

  return pure_controller_->queryStateService(req, resp);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>::
trajectoryCommandCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  if (!this->isRunning())
  {
    ROS_ERROR_STREAM_NAMED(controller_name_, "Can't accept new commands. Controller is not running.");
    return;
  }

  pure_controller_->trajectoryCommandCB(msg);
}

}

#endif // JOINT_TRAJECTORY_CONTROLLER_IMPL_REVISED
