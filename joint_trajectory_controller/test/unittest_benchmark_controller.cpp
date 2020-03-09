///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, HiddenBug
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

#include <memory>

#include <benchmark/benchmark.h>

#include <hardware_interface/joint_command_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <joint_trajectory_controller/pure_joint_trajectory_controller.h>

using HWInterface = hardware_interface::JointCommandInterface;

using Segment = trajectory_interface::QuinticSplineSegment<double>;
using State = typename Segment::State;
using PureController = pure_joint_trajectory_controller::PureJointTrajectoryController<State>;

/**
 * @brief Measures the execution time of the update function of the controller.
 */
static void BM_UpdateFunc(benchmark::State& state)
{
  const ros::Time time {0.024};
  const ros::Duration periode {0.008};
  std::unique_ptr<PureController > controller = std::unique_ptr<PureController>(new PureController());

  for (auto _ : state)
  {
    controller->update(time, periode);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_UpdateFunc);

BENCHMARK_MAIN();
