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

#ifndef JOINT_TRAECTORY_CONTROLLER_REVISED_H
#define JOINT_TRAECTORY_CONTROLLER_REVISED_H

#include <controller_interface/controller.h>

// ROS
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <joint_trajectory_controller/pure_joint_trajectory_controller.h>

namespace joint_trajectory_controller_revised
{

template <class SegmentImpl, class HardwareInterface>
class JointTrajectoryControllerRevised: public controller_interface::Controller<HardwareInterface>
{
public:
  JointTrajectoryControllerRevised()
    : controller_interface::Controller<HardwareInterface>()
  {

  }

  virtual ~JointTrajectoryControllerRevised<SegmentImpl, HardwareInterface>() = default;

public:
  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /*\}*/

public:
  /** \name Real-Time Safe Functions
   *\{*/
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;
  /*\}*/

};

}

#include<joint_trajectory_controller/joint_trajectory_controller_impl_revised.h>

#endif
