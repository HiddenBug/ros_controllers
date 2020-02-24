///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Immanuel Martini

#include <gtest/gtest.h>

#include <joint_trajectory_controller/stop_trajectory_builder.h>

#include <trajectory_interface/quintic_spline_segment.h>

#include "test_common.h"

namespace stop_trajectory_builder_test
{
static constexpr double EPS{1e-9};

template<class SegmentType>
bool statesAlmostEqual(const typename SegmentType::State& state1,
                       const typename SegmentType::State& state2,
                       const double& tolerance=EPS)
{
  using namespace joint_trajectory_controller_tests;
  return vectorsAlmostEqual(state1.position, state2.position, tolerance) &&
         vectorsAlmostEqual(state1.velocity, state2.velocity, tolerance) &&
         vectorsAlmostEqual(state1.acceleration, state2.acceleration, tolerance);
}

using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
using TrajectoryPerJoint = std::vector<Segment>;
using Trajectory = std::vector<TrajectoryPerJoint>;

using GoalHandle = actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle;
using RealTimeServerGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;

class StopTrajectoryBuilderTest : public testing::Test
{
public:
  void initDefaultTrajectory(unsigned int number_of_joints, Trajectory& trajectory)
  {
    Segment::State state{1};
	  Segment segment(0.0, state, 1.0, state);
	  TrajectoryPerJoint joint_traj{segment};
    trajectory.resize(number_of_joints, joint_traj);
  }
};

TEST_F(StopTrajectoryBuilderTest, testBuildNoStartTime)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	unsigned int number_of_joints{2};
	double stop_trajectory_duration{0.2};
  Segment::State hold_state{number_of_joints};
  Builder builder(number_of_joints, stop_trajectory_duration, hold_state);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite start time is not set.";
}

TEST_F(StopTrajectoryBuilderTest, testBuildSuccess)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	unsigned int number_of_joints{2};
	double stop_duration{0.2};
  double start_time{0.11};
  Segment::State hold_state{number_of_joints};
	hold_state.position[0] = 0.9;  // set arbitrary state
	hold_state.velocity[0] = -0.03;
	hold_state.position[1] = 1.68;
	hold_state.velocity[1] = 3.7;
  Builder builder(number_of_joints, stop_duration, hold_state);

	builder.setStartTime(start_time);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory));

	// check built trajectory
  EXPECT_EQ(trajectory.size(), number_of_joints);
  for (const auto& jt : trajectory)
  {
    EXPECT_EQ(jt.size(), 1U);
  }
  EXPECT_NEAR(trajectory[0][0].startTime(), start_time, EPS);
	EXPECT_NEAR(trajectory[0][0].endTime(), start_time + stop_duration, EPS);

  // check start and end state
  Segment::State sampled_state{1};
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    trajectory[i][0].sample(trajectory[i][0].startTime(), sampled_state);
		EXPECT_NEAR(sampled_state.position[0], hold_state.position[i], EPS);
		EXPECT_NEAR(sampled_state.velocity[0], hold_state.velocity[i], EPS);

    trajectory[i][0].sample(trajectory[i][0].endTime(), sampled_state);
		EXPECT_NEAR(sampled_state.velocity[0], 0.0, EPS);
	}
}

TEST_F(StopTrajectoryBuilderTest, testResetStartTime)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	unsigned int number_of_joints{2};
	double stop_duration{0.2};
  double start_time{0.11};
  Segment::State hold_state{number_of_joints};

  Builder builder(number_of_joints, stop_duration, hold_state);
	builder.setStartTime(start_time);

	builder.reset();

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory));
}

/**
 * @note A non-empty goal handle cannot be created without an action server,
 * therefore we check only how the use_count of the shared pointer changes.
 */
TEST_F(StopTrajectoryBuilderTest, testSetGoalHandle)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	unsigned int number_of_joints{2};
	double stop_duration{0.2};
  double start_time{0.11};
  Segment::State hold_state{number_of_joints};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  EXPECT_EQ(rt_goal_handle.use_count(), 1);

  Builder builder(number_of_joints, stop_duration, hold_state);
	builder.setStartTime(start_time);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory));

  EXPECT_EQ(rt_goal_handle.use_count(), 3);
}

/**
 * @note A non-empty goal handle cannot be created without an action server,
 * therefore we check only how the use_count of the shared pointer changes.
 */
TEST_F(StopTrajectoryBuilderTest, testResetGoalHandle)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	unsigned int number_of_joints{2};
	double stop_duration{0.2};
  double start_time{0.11};
  Segment::State hold_state{number_of_joints};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  EXPECT_EQ(rt_goal_handle.use_count(), 1);

  Builder builder(number_of_joints, stop_duration, hold_state);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1);

	builder.reset();
	builder.setStartTime(start_time);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory));

  EXPECT_EQ(rt_goal_handle.use_count(), 1);
}

}  // namespace stop_trajectory_builder_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
