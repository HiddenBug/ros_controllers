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

#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>

#include <joint_trajectory_controller/hold_trajectory_builder.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

#include <trajectory_interface/quintic_spline_segment.h>

#include "test_common.h"

namespace hold_trajectory_builder_test
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

using JointHandle = hardware_interface::JointHandle;
using PosVelJointHandle = hardware_interface::PosVelJointHandle;
using PosVelAccJointHandle = hardware_interface::PosVelAccJointHandle;
using JointStateHandle = hardware_interface::JointStateHandle;
using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
using TrajectoryPerJoint = std::vector<Segment>;
using Trajectory = std::vector<TrajectoryPerJoint>;

/**
 * @brief Provides all known hardware interfaces.
 */
class FakeRobot
{
public:
  FakeRobot()
    : jsh1_("joint1", &pos1_, &vel2_, &eff1_),
      jsh2_("joint2", &pos2_, &vel2_, &eff2_)
  {
  }

  template<typename HardwareInterface>
  std::vector<typename HardwareInterface::ResourceHandleType> getJointHandles();

  void setPositions(const double& pos1, const double& pos2)
  {
    pos1_ = pos1;
    pos2_ = pos2;
  }

private:
  std::vector<JointHandle> getJointCommandHandles()
  {
	  JointHandle joint1(jsh1_, &cmd1_);
	  JointHandle joint2(jsh2_, &cmd2_);
    return {joint1, joint2};
  }

private:
	double pos1_, vel1_, eff1_, cmd1_, vel_cmd1_, acc_cmd1_ = 0.0;
	double pos2_, vel2_, eff2_, cmd2_, vel_cmd2_, acc_cmd2_ = 0.0;
  JointStateHandle jsh1_, jsh2_;
};

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::PositionJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::VelocityJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::EffortJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<PosVelJointHandle> FakeRobot::getJointHandles<hardware_interface::PosVelJointInterface>()
{
	PosVelJointHandle joint1(jsh1_, &cmd1_, &vel_cmd1_);
	PosVelJointHandle joint2(jsh2_, &cmd2_, &vel_cmd2_);
  return {joint1, joint2};
}

template<>
std::vector<PosVelAccJointHandle> FakeRobot::getJointHandles<hardware_interface::PosVelAccJointInterface>()
{
	PosVelAccJointHandle joint1(jsh1_, &cmd1_, &vel_cmd1_, &acc_cmd1_);
	PosVelAccJointHandle joint2(jsh2_, &cmd2_, &vel_cmd2_, &acc_cmd2_);
  return {joint1, joint2};
}

template<typename HardwareInterface>
class HoldTrajectoryBuilderTest : public testing::Test
{
public:
  void initDefaultTrajectory(unsigned int number_of_joints, Trajectory& trajectory)
  {
    Segment::State state{1};
	  Segment segment(0.0, state, 1.0, state);
	  TrajectoryPerJoint joint_traj{segment};
    trajectory.resize(number_of_joints, joint_traj);
  }

  std::vector<typename HardwareInterface::ResourceHandleType> getJointHandles()
  {
    return robot_.getJointHandles<HardwareInterface>();
  }

public:
  FakeRobot robot_;
};

using HardwareInterfaceTypes = testing::Types<hardware_interface::PositionJointInterface,
                                              hardware_interface::VelocityJointInterface,
                                              hardware_interface::EffortJointInterface,
                                              hardware_interface::PosVelJointInterface,
                                              hardware_interface::PosVelAccJointInterface>;

TYPED_TEST_CASE(HoldTrajectoryBuilderTest, HardwareInterfaceTypes);

TYPED_TEST(HoldTrajectoryBuilderTest, testBuildNoStartTime)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  auto number_of_joints = joints.size();

  Builder builder(number_of_joints, joints);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite start time is not set.";	
}

TYPED_TEST(HoldTrajectoryBuilderTest, testBuildSuccess)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  auto number_of_joints = joints.size();
  double start_time{0.0};

  Builder builder(number_of_joints, joints);
	builder.setStartTime(start_time);

  Trajectory trajectory;
  this->initDefaultTrajectory(number_of_joints, trajectory);

  std::vector<double> positions{-48.9, 2.1};  // set some arbitrary positions
  this->robot_.setPositions(positions[0], positions[1]);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory));

  // check built trajectory
  EXPECT_EQ(trajectory.size(), number_of_joints);
  for (const auto& jt : trajectory)
  {
    EXPECT_EQ(jt.size(), 1U);
  }
  EXPECT_NEAR(trajectory[0][0].startTime(), start_time, EPS);

  // check start and end state
  Segment::State sampled_state{1};
  Segment::State expected_state{1};
  expected_state.velocity[0] = 0.0;
  expected_state.acceleration[0] = 0.0;
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    expected_state.position[0] = positions[i];

    trajectory[i][0].sample(trajectory[i][0].startTime(), sampled_state);
    EXPECT_TRUE(statesAlmostEqual<Segment>(sampled_state, expected_state));

    trajectory[i][0].sample(trajectory[i][0].endTime(), sampled_state);
    EXPECT_TRUE(statesAlmostEqual<Segment>(sampled_state, expected_state));
  }
}

}  // namespace hold_trajectory_builder_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
