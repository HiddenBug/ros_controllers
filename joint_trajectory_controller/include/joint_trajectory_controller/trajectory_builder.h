///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef TRAJECTORY_BUILDER_H
#define TRAJECTORY_BUILDER_H

#include <vector>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>

// realtime_tools
#include <realtime_tools/realtime_server_goal_handle.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace joint_trajectory_controller
{

/**
 * @brief Base class for classes used to construct diffent trajectory types.
 */
template<class SegmentImpl>
class TrajectoryBuilder
{
private:
  using Segment               = JointTrajectorySegment<SegmentImpl>;
  using TrajectoryPerJoint    = std::vector<Segment>;
  using Trajectory            = std::vector<TrajectoryPerJoint>;

  using RealtimeGoalHandle    = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;
  using RealtimeGoalHandlePtr = boost::shared_ptr<RealtimeGoalHandle>;

public:
  TrajectoryBuilder<SegmentImpl>* setStartTime(const typename Segment::Time& start_time);
  TrajectoryBuilder<SegmentImpl>* setGoalHandle(RealtimeGoalHandlePtr goal_handle);

public:
  /**
   * @brief Ensures that builder does not influence life cycle of external variables
   * by reseting all essential class members.
   */
  virtual void reset();

public: 
  /**
   * @brief Creates the type of trajectory described by the builder.
   *
   * @param trajectory [Out] Trajectory which has to be build.
   *
   */
  virtual bool buildTrajectory(Trajectory* trajectory) = 0;

protected:
  RealtimeGoalHandlePtr getGoalHandle() const;
  const boost::optional<typename Segment::Time>& getStartTime() const;

private:
  boost::optional<typename Segment::Time> start_time_   {boost::none};
  boost::optional<RealtimeGoalHandlePtr> goal_handle_         {boost::none};

};

template<class SegmentImpl>
inline TrajectoryBuilder<SegmentImpl>* TrajectoryBuilder<SegmentImpl>::setStartTime(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time)
{
  start_time_ = start_time;
  return this;
}

template<class SegmentImpl>
inline const boost::optional<typename TrajectoryBuilder<SegmentImpl>::Segment::Time>& TrajectoryBuilder<SegmentImpl>::getStartTime() const
{
  return start_time_;
}

template<class SegmentImpl>
inline TrajectoryBuilder<SegmentImpl>* TrajectoryBuilder<SegmentImpl>::setGoalHandle(TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr goal_handle)
{
  goal_handle_ = goal_handle;
  return this;
}

template<class SegmentImpl>
inline typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr TrajectoryBuilder<SegmentImpl>::getGoalHandle() const
{
  return goal_handle_ ? goal_handle_.value() : RealtimeGoalHandlePtr();
}

template<class SegmentImpl>
inline void TrajectoryBuilder<SegmentImpl>::reset()
{
  start_time_ = boost::none;
  goal_handle_ = boost::none;
}


}

#endif // TRAJECTORY_BUILDER_H
