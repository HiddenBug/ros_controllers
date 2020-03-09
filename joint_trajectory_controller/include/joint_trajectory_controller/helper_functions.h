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

#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

// C++ standard
#include <stdexcept>
#include <vector>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>

// URDF
#include <urdf/model.h>

// ROS
#include <ros/node_handle.h>

namespace joint_trajectory_controller
{

namespace internal
{

template <class Enclosure, class Member>
inline boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    XmlRpc::XmlRpcValue& elem = xml_array[i];
    if (elem.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(elem));
  }
  return out;
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  urdf::ModelSharedPtr urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str))
  {
    if (!urdf->initString(urdf_str))
    {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
        nh.getNamespace() << ").");
      return urdf::ModelSharedPtr();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description"))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
    return urdf::ModelSharedPtr();
  }
  return urdf;
}

std::vector<urdf::JointConstSharedPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
{
  std::vector<urdf::JointConstSharedPtr> out;
  for (const auto& joint_name : joint_names)
  {
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in URDF model.");
      return std::vector<urdf::JointConstSharedPtr>();
    }
  }
  return out;
}

std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

class UrdfModelMissing : public std::runtime_error
{
  public:
    UrdfModelMissing():
      std::runtime_error("URDF model missing")
    {
    }
};

class NumberOfUrdfJointsIncorrect : public std::runtime_error
{
  public:
    NumberOfUrdfJointsIncorrect():
      std::runtime_error("Number of joints in URDF model incorrect")
    {
    }
};

static std::vector<bool> getAngleWraparound(ros::NodeHandle& root_nh,
                                            const std::vector<std::string>& joint_names)
{
  std::vector<bool> angle_wraparound;
  angle_wraparound.resize(joint_names.size());

  urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
  if (!urdf)
  {
    throw UrdfModelMissing();
  }

  std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, joint_names);
  if ( urdf_joints.empty() || (joint_names.size() != urdf_joints.size()) )
  {
    throw NumberOfUrdfJointsIncorrect();
  }

  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    angle_wraparound.at(i) = urdf_joints.at(i)->type == urdf::Joint::CONTINUOUS;
  }

  return angle_wraparound;
}

} // namespace

}

#endif // HELPER_FUNCTIONS_H
