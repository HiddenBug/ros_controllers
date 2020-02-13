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
#ifndef HOLD_TRAJECTORY_MANAGER_H
#define HOLD_TRAJECTORY_MANAGER_H

#include <vector>
#include <memory>

#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace joint_trajectory_controller
{

template<class SegmentImpl>
class HoldTrajectoryManager
{
private:
  using Segment               = JointTrajectorySegment<SegmentImpl>;

  using TrajectoryPerJoint    = std::vector<Segment>;
  using Trajectory            = std::vector<TrajectoryPerJoint>;
  using TrajectoryPtr         = std::shared_ptr<Trajectory>;

public:
  HoldTrajectoryManager(const unsigned int& number_of_joints);

public:
  TrajectoryPtr& getHoldTrajectory();

private:
  TrajectoryPtr hold_trajectory_ptr_ {new Trajectory};
};

template<class SegmentImpl>
HoldTrajectoryManager<SegmentImpl>::HoldTrajectoryManager(const unsigned int& number_of_joints)
{
  typename Segment::State default_state       = typename Segment::State(number_of_joints);
  typename Segment::State default_joint_state = typename Segment::State(1);
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    default_joint_state.position[0]= default_state.position[i];
    default_joint_state.velocity[0]= default_state.velocity[i];
    Segment hold_segment(0.0, default_joint_state, 0.0, default_joint_state);

    TrajectoryPerJoint joint_segment;
    joint_segment.resize(1, hold_segment);
    hold_trajectory_ptr_->push_back(joint_segment);
  }
}

template<class SegmentImpl>
inline typename HoldTrajectoryManager<SegmentImpl>::TrajectoryPtr& HoldTrajectoryManager<SegmentImpl>::getHoldTrajectory()
{
  return hold_trajectory_ptr_;
}

}


#endif // HOLD_TRAJECTORY_MANAGER_H
