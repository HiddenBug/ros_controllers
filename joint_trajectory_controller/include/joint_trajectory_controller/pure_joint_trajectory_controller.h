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

#ifndef PURE_JOINT_TRAJECTORY_CONTROLLER_H
#define PURE_JOINT_TRAJECTORY_CONTROLLER_H

// C++ standard
#include <string>
#include <vector>
#include <memory>

// ROS
#include <ros/time.h>
#include <ros/duration.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

// Project
#include <joint_trajectory_controller/tolerances.h>

namespace pure_joint_trajectory_controller
{

/**
 * @brief Stores settings which configures the HWInputHandle.
 */
class HWInputHandleSettings
{
public:
  virtual ~HWInputHandleSettings() = default;

public:
  virtual const std::vector<bool>& getAngleWrapAround() const = 0;
  virtual const std::vector<std::string>& getJointNames() const = 0;
  virtual const unsigned int getNumberOfJoints() const = 0;
};

class EditableHWInputHandleSettings : public HWInputHandleSettings
{
public:
  EditableHWInputHandleSettings();

public:
  const std::vector<bool>& getAngleWrapAround() const override;
  const std::vector<std::string>& getJointNames() const override;
  const unsigned int getNumberOfJoints() const override;

public:
  void setAngleWrapAround(const std::vector<bool>& angle_wrap_around);
  void setJointNames(const std::vector<std::string>& joint_names);

private:
  std::vector<bool> angle_wrap_around_;
  std::vector<std::string> joint_names_;
};

/**
 * @brief Interface to obtain information from the hardware interface.
 */
template<class State>
class HWInputHandle
{
public:
  HWInputHandle(std::unique_ptr<HWInputHandleSettings> settings);
  virtual ~HWInputHandle() = default;

public:
  virtual const State& getState() const = 0;

public:
  const HWInputHandleSettings* const getSettings() const;

private:
  std::unique_ptr<HWInputHandleSettings> settings_;
};

template<class State, class JointHandle>
class JointHandleHWInputHandle : public HWInputHandle<State>
{
public:
  JointHandleHWInputHandle(std::unique_ptr<HWInputHandleSettings> settings,
                           const std::vector<JointHandle>& joint_handles);

public:
  const State& getState() const override;

private:
  //! We do not participate in the life time management!
  const std::vector<JointHandle>& joint_handles_;
};

/**
 * @brief Interface to update the hardware interface.
 */
template<class State>
class HWOutputHandle
{
public:
  virtual ~HWOutputHandle() = default;

public:
  virtual void updateHardware(const ros::Time& uptime, const ros::Duration& period,
                              const State& desired, const State& error) = 0;

};

template<class State, class HwIfaceAdapter>
class HWOutputHandleForHWInterfaceAdapter : public HWOutputHandle<State>
{
public:
  HWOutputHandleForHWInterfaceAdapter(std::unique_ptr<HwIfaceAdapter> adapter);

public:
  void updateHardware(const ros::Time& uptime, const ros::Duration& period,
                      const State& desired, const State& error) override;

private:
  std::unique_ptr<HwIfaceAdapter> adapter_;
};

/**
 * @brief Interface to inform ROS about changes in states, etc.
 */
template<class State>
class ROSOutputHandle
{
public:
  virtual ~ROSOutputHandle() = default;

public:
  virtual void sentUpdate(const State& current, const State& desired, const State& error) = 0;
};

/**
 * @brief Interface for storing and combining controller settings.
 */
template<class SegmentTolerances>
class ControllerSettings
{
public:
  virtual ~ControllerSettings() = default;

public:
  virtual const std::string& getControllerName() const = 0;
  virtual const ros::Duration& getStatePublishRate() const = 0;
  virtual const ros::Duration& getActionMonitorRate() const = 0;
  virtual const double& getStopTrajectoryDuration() const = 0;
  virtual const SegmentTolerances& getDefaultSegmentTolerances() const = 0;
  virtual const bool& arePartialJointGoalsAllowed() const = 0;
};

template<class SegmentTolerances>
class EditableControllerSettings : public ControllerSettings<SegmentTolerances>
{
public:
  EditableControllerSettings();

public:
  void setControllerName(const std::string& name);
  void setStatePublishRate(const ros::Duration& rate);
  void setActionMonitorRate(const ros::Duration& rate);
  void setStopTrajectoryDuration(const double& duration);
  void setDefaultSegmentTolerances(const SegmentTolerances& tolerances);
  void setPartialJointGoals(const bool& partial_goals);

public:
  const std::string& getControllerName() const override;
  const ros::Duration &getStatePublishRate() const override;
  const ros::Duration& getActionMonitorRate() const override;
  const double& getStopTrajectoryDuration() const override;
  const SegmentTolerances& getDefaultSegmentTolerances() const override;
  const bool& arePartialJointGoalsAllowed() const override;

private:
  std::string controller_name_;
  ros::Duration state_publish_rate_ {0.};
  ros::Duration action_monitor_rate_ {0.};
  double stop_traj_duration_ {0.};
  SegmentTolerances default_tolerances_;
  bool partial_joint_goals_allowed_ {false};
};

template<class State>
class PureJointTrajectoryController
{
private:
  using ActionServer  = actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>;
  using GoalHandle    = ActionServer::GoalHandle;

public:
  PureJointTrajectoryController()
  {

  }

  PureJointTrajectoryController(std::unique_ptr<HWInputHandle<State> >  hw_input,
                                std::unique_ptr<HWOutputHandle<State> > hw_output)
  {

  }

public:
  /** \name Real-Time Safe Functions
   *\{*/
  void starting(const ros::Time& time)  {};
  void stopping(const ros::Time& /*time*/) {};

  void update(const ros::Time& time, const ros::Duration& period)  {};
  /*\}*/

public: // ROS callback and service functions
  void trajectoryCommandCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {};
  void goalCB(GoalHandle gh)  {};
  void cancelCB(GoalHandle gh)  {};

  bool queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                         control_msgs::QueryTrajectoryState::Response& resp)  {};

};

}

#include <joint_trajectory_controller/pure_joint_trajectory_controller_impl.h>

#endif // PURE_JOINT_TRAJECTORY_CONTROLLER_H
