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

#include <string>
#include <vector>
#include <memory>

#include <ros/time.h>
#include <ros/duration.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

namespace pure_joint_trajectory_controller
{

/**
 * @brief Interface to obtain information from the hardware interface.
 */
template<class State>
class HWInputHandle
{
public:
  virtual ~HWInputHandle() = default;

public:
  virtual const State& getState() const = 0;

public:
  virtual const std::vector<bool>& getAngleWrapAround() const = 0;
  virtual const std::vector<std::string>& getJointNames() const = 0;
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
  virtual void setState(const State& desired, const State& error) = 0;

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
class ControllerSettings
{
public:
  virtual ~ControllerSettings() = default;

public:
  virtual const std::string& getControllerName() const = 0;
  virtual const double& getStatePublishRate() const = 0;
  virtual const double& getActionMonitorRate() const = 0;
  virtual const double& getStopTrajectoryDuration() const = 0;
};

class EditableControllerSettings : public ControllerSettings
{
public:
  EditableControllerSettings();

public:
  void setControllerName(const std::string& name);
  void setStatePublishRate(const double& rate);
  void setActionMonitorRate(const double& rate);
  void setStopTrajectoryDuration(const double& duration);

public:
  const std::string& getControllerName() const override;
  const double& getStatePublishRate() const override;
  const double& getActionMonitorRate() const override;
  const double& getStopTrajectoryDuration() const override;

private:
  std::string controller_name_;
  double state_publish_rate_ {0.};
  double action_monitor_rate_ {0.};
  double stop_traj_duration_ {0.};
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
                                std::unique_ptr<HWOutputHandle<State> > hw_output,
                                std::unique_ptr<ControllerSettings> settings)
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


inline EditableControllerSettings::EditableControllerSettings()
  : ControllerSettings()
{

}

inline void EditableControllerSettings::setControllerName(const std::string& name)
{
  controller_name_ = name;
}

inline void EditableControllerSettings::setStatePublishRate(const double& rate)
{
  state_publish_rate_ = rate;
}

inline void EditableControllerSettings::setActionMonitorRate(const double& rate)
{
  action_monitor_rate_ = rate;
}

inline void EditableControllerSettings::setStopTrajectoryDuration(const double& duration)
{
  stop_traj_duration_ = duration;
}

inline const std::string& EditableControllerSettings::getControllerName() const
{
  return controller_name_;
}

inline const double& EditableControllerSettings::getStatePublishRate() const
{
  return state_publish_rate_;
}

inline const double& EditableControllerSettings::getActionMonitorRate() const
{
  return action_monitor_rate_;
}

inline const double& EditableControllerSettings::getStopTrajectoryDuration() const
{
  return stop_traj_duration_;
}

}

#endif // PURE_JOINT_TRAJECTORY_CONTROLLER_H
