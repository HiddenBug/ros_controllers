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
///
#ifndef PURE_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
#define PURE_JOINT_TRAJECTORY_CONTROLLER_IMPL_H

namespace pure_joint_trajectory_controller
{

inline EditableHWInputHandleSettings::EditableHWInputHandleSettings()
  : HWInputHandleSettings()
{
}

inline const std::vector<bool>& EditableHWInputHandleSettings::getAngleWrapAround() const
{
  return angle_wrap_around_;
}

inline const std::vector<std::string>& EditableHWInputHandleSettings::getJointNames() const
{
  return joint_names_;
}

inline void EditableHWInputHandleSettings::setAngleWrapAround(const std::vector<bool>& angle_wrap_around)
{
  angle_wrap_around_ = angle_wrap_around;
}

inline void EditableHWInputHandleSettings::setJointNames(const std::vector<std::string>& joint_names)
{
  joint_names_ = joint_names;
}

inline const unsigned int EditableHWInputHandleSettings::getNumberOfJoints() const
{
  return joint_names_.size();
}

template<class State>
inline HWInputHandle<State>::HWInputHandle(std::unique_ptr<HWInputHandleSettings> settings)
  : settings_(std::move(settings))
{

}

template<class State>
inline const HWInputHandleSettings* const HWInputHandle<State>::getSettings() const
{
  return settings_.get();
}

template<class State, class JointHandle>
inline JointHandleHWInputHandle<State, JointHandle>::JointHandleHWInputHandle(std::unique_ptr<HWInputHandleSettings> settings,
                                                                              const std::vector<JointHandle>& joint_handles)
  : HWInputHandle<State>(std::move(settings))
  , joint_handles_(joint_handles)
{
}

template<class State, class JointHandle>
inline const State& JointHandleHWInputHandle<State, JointHandle>::getState() const
{
  static State current_state { State(HWInputHandle<State>::getSettings()->getNumberOfJoints()) };

  for (unsigned int joint_index = 0; joint_index < HWInputHandle<State>::getSettings()->getNumberOfJoints(); ++joint_index)
  {
    current_state.position[joint_index] = joint_handles_[joint_index].getPosition();
    current_state.velocity[joint_index] = joint_handles_[joint_index].getVelocity();
    // There's no acceleration data available in a joint handle
  }
  return current_state;
}

template<class SegmentTolerances>
inline EditableControllerSettings<SegmentTolerances>::EditableControllerSettings()
  : ControllerSettings<SegmentTolerances>()
{
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setControllerName(const std::string& name)
{
  controller_name_ = name;
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setStatePublishRate(const ros::Duration &rate)
{
  state_publish_rate_ = rate;
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setActionMonitorRate(const ros::Duration& rate)
{
  action_monitor_rate_ = rate;
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setStopTrajectoryDuration(const double& duration)
{
  stop_traj_duration_ = duration;
}

template<class SegmentTolerances>
inline const std::string& EditableControllerSettings<SegmentTolerances>::getControllerName() const
{
  return controller_name_;
}

template<class SegmentTolerances>
inline const ros::Duration& EditableControllerSettings<SegmentTolerances>::getStatePublishRate() const
{
  return state_publish_rate_;
}

template<class SegmentTolerances>
inline const ros::Duration& EditableControllerSettings<SegmentTolerances>::getActionMonitorRate() const
{
  return action_monitor_rate_;
}

template<class SegmentTolerances>
inline const double& EditableControllerSettings<SegmentTolerances>::getStopTrajectoryDuration() const
{
  return stop_traj_duration_;
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setDefaultSegmentTolerances(const SegmentTolerances& tolerances)
{
  default_tolerances_ = tolerances;
}

template<class SegmentTolerances>
inline const SegmentTolerances& EditableControllerSettings<SegmentTolerances>::getDefaultSegmentTolerances() const
{
  return default_tolerances_;
}

template<class SegmentTolerances>
inline void EditableControllerSettings<SegmentTolerances>::setPartialJointGoals(const bool& partial_goals)
{
  partial_joint_goals_allowed_ = partial_goals;
}

template<class SegmentTolerances>
inline const bool& EditableControllerSettings<SegmentTolerances>::arePartialJointGoalsAllowed() const
{
  return partial_joint_goals_allowed_;
}

template<class State, class HwIfaceAdapter>
HWOutputHandleForHWInterfaceAdapter<State, HwIfaceAdapter>::HWOutputHandleForHWInterfaceAdapter(std::unique_ptr<HwIfaceAdapter> adapter)
  : adapter_(std::move(adapter))
{

}

template<class State, class HwIfaceAdapter>
inline void HWOutputHandleForHWInterfaceAdapter<State, HwIfaceAdapter>::
updateHardware(const ros::Time& uptime, const ros::Duration& period, const State& desired, const State& error)
{
  adapter_->updateCommand(uptime, period, desired, error);
}

}

#endif // PURE_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
