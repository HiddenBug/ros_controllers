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
#ifndef HOLD_TRAJECTORY_BUILDER_H
#define HOLD_TRAJECTORY_BUILDER_H

#include <vector>

#include <joint_trajectory_controller/trajectory_builder.h>

namespace joint_trajectory_controller
{

/**
 * @brief Builder creating a trajectory "simply" holding (without motion)
 * the specified position.
 */
template<class SegmentImpl, class HardwareInterface>
class HoldTrajectoryBuilder : public TrajectoryBuilder<SegmentImpl>
{
private:
  using JointHandle = typename HardwareInterface::ResourceHandleType;

public:
  HoldTrajectoryBuilder(const unsigned int& number_of_joints,
                        const std::vector<JointHandle>&  joints);

public:
  bool buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                       typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                       typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle) override;

private:
  const unsigned int number_of_joints_;
  const std::vector<JointHandle>&  joints_;

private: //Pre-allocated memory for real time usage of build function
  typename TrajectoryBuilder<SegmentImpl>::Segment::State hold_start_state_ {typename TrajectoryBuilder<SegmentImpl>::Segment::State(1)};
  typename TrajectoryBuilder<SegmentImpl>::Segment::State hold_end_state_   {typename TrajectoryBuilder<SegmentImpl>::Segment::State(1)};

};

template<class SegmentImpl, class HardwareInterface>
HoldTrajectoryBuilder<SegmentImpl, HardwareInterface>::HoldTrajectoryBuilder(const unsigned int& number_of_joints,
                                                                             const std::vector<JointHandle>& joints)
  : number_of_joints_(number_of_joints)
  , joints_(joints)
{

}

template<class SegmentImpl, class HardwareInterface>
bool HoldTrajectoryBuilder<SegmentImpl, HardwareInterface>::buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                                                                            typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                                                                            typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle)
{
  for (unsigned int joint_index = 0; joint_index < number_of_joints_; ++joint_index)
  {
    hold_start_state_.position[0]     =  joints_[joint_index].getPosition();
    hold_start_state_.velocity[0]     =  0.0;
    hold_start_state_.acceleration[0] =  0.0;

    typename TrajectoryBuilder<SegmentImpl>::Segment& segment {(*hold_traj)[joint_index].front()};
    segment.init(start_time, hold_start_state_,
                 start_time, hold_start_state_);
    segment.setGoalHandle(goal_handle);
  }
  return true;
}

}

#endif // HOLD_TRAJECTORY_BUILDER_H
