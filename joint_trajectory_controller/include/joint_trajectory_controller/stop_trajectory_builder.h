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
#ifndef STOP_TRAJECTORY_BUILDER_H
#define STOP_TRAJECTORY_BUILDER_H

#include <joint_trajectory_controller/trajectory_builder.h>

namespace joint_trajectory_controller
{

/**
 * @brief Builder creating a trajectory stopping the robot.
 *
 * @note The start state of the stop motion and the final settle/hold state
 * are eqal.
 */
template<class SegmentImpl>
class StopTrajectoryBuilder : public TrajectoryBuilder<SegmentImpl>
{
public:
  StopTrajectoryBuilder(const unsigned int& number_of_joints,
                        const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& stop_traj_duration);

public:

  bool buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                       const typename TrajectoryBuilder<SegmentImpl>::Segment::State& start_state,
                       typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                       typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle);

  /**
   * @brief Creates a trajectory which reaches the settle position in a fixed time.
   *
   * The calculation is done as follows:
   * - Create segment that goes from current (pos,vel) to (pos,-vel) in 2x the desired stop time.
   * - Assuming segment symmetry, sample segment at its midpoint (desired stop time). It should have zero velocity.
   * - Create segment that goes from current state to above zero velocity state, in the desired time.
   *
   * @note The symmetry assumption from the second point above might not hold for all possible segment types.
   *
   * @returns true if the building of the stop trajectory succeeds, otherwise false.
   */
  bool buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                       typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                       typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle) override;

public:
    void setStartState(const typename TrajectoryBuilder<SegmentImpl>::Segment::State& start_state);

private:
  const unsigned int number_of_joints_;
  const typename TrajectoryBuilder<SegmentImpl>::Segment::Time stop_traj_duration_;

private:
  boost::optional<const typename TrajectoryBuilder<SegmentImpl>::Segment::State&> start_state_       {boost::none};

private: //Pre-allocated memory for real time usage of build function
  typename TrajectoryBuilder<SegmentImpl>::Segment::State hold_start_state_ {typename TrajectoryBuilder<SegmentImpl>::Segment::State(1)};
  typename TrajectoryBuilder<SegmentImpl>::Segment::State hold_end_state_   {typename TrajectoryBuilder<SegmentImpl>::Segment::State(1)};
};

template<class SegmentImpl>
StopTrajectoryBuilder<SegmentImpl>::StopTrajectoryBuilder(const unsigned int& number_of_joints,
                                                          const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& stop_traj_duration)
  : number_of_joints_(number_of_joints)
  , stop_traj_duration_(stop_traj_duration)
{
}

template<class SegmentImpl>
inline void StopTrajectoryBuilder<SegmentImpl>::setStartState(const typename TrajectoryBuilder<SegmentImpl>::Segment::State& start_state)
{
  start_state_ = start_state;
}

template<class SegmentImpl>
bool StopTrajectoryBuilder<SegmentImpl>::buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                                                         const typename TrajectoryBuilder<SegmentImpl>::Segment::State& start_state,
                                                         typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                                                         typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle)
{
  const typename TrajectoryBuilder<SegmentImpl>::Segment::Time end_time    {start_time + stop_traj_duration_};
  const typename TrajectoryBuilder<SegmentImpl>::Segment::Time end_time_2x {start_time + 2.0 * stop_traj_duration_};
  for (unsigned int joint_index = 0; joint_index < number_of_joints_; ++joint_index)
  {
    // If there is a time delay in the system it is better to calculate the hold trajectory starting from the
    // desired position. Otherwise there would be a jerk in the motion.
    hold_start_state_.position[0]     =  start_state.position[joint_index];
    hold_start_state_.velocity[0]     =  start_state.velocity[joint_index];
    hold_start_state_.acceleration[0] =  0.0;

    hold_end_state_.position[0]       =  start_state.position[joint_index];
    hold_end_state_.velocity[0]       = -start_state.velocity[joint_index];
    hold_end_state_.acceleration[0]   =  0.0;

    typename TrajectoryBuilder<SegmentImpl>::Segment& segment {(*hold_traj)[joint_index].front()};
    segment.init(start_time,  hold_start_state_,
                 end_time_2x, hold_end_state_);
    // Sample segment at its midpoint, that should have zero velocity
    segment.sample(end_time, hold_end_state_);
    // Now create segment that goes from current state to one with zero end velocity
    segment.init(start_time,  hold_start_state_,
                 end_time,    hold_end_state_);
    segment.setGoalHandle(goal_handle);
  }

  return true;
}

template<class SegmentImpl>
inline bool StopTrajectoryBuilder<SegmentImpl>::buildTrajectory(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time,
                                                                typename TrajectoryBuilder<SegmentImpl>::Trajectory* hold_traj,
                                                                typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle)
{
  if (!start_state_)
  {
    return false;
  }
  return buildTrajectory(start_time, start_state_.value(), hold_traj, goal_handle);
}


} // namespace joint_trajectory_controller

#endif // STOP_TRAJECTORY_BUILDER_H
