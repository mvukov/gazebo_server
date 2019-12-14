// Copyright 2019 Milan Vukov. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef GAZEBO_SERVER_HELPERS_H_
#define GAZEBO_SERVER_HELPERS_H_

#include <cmath>
#include <string>

#include <Eigen/Geometry>

namespace gazebo_server {

inline Eigen::Vector3d DcmToEulerAngles(const Eigen::Matrix3d& global_r_local) {
  const auto r11 = global_r_local(0, 0);
  const auto r21 = global_r_local(1, 0);
  auto mr31 = -global_r_local(2, 0);
  const auto r32 = global_r_local(2, 1);
  const auto r33 = global_r_local(2, 2);

  Eigen::Vector3d rpy;
  rpy(2) = std::atan2(r21, r11);
  if (mr31 > 1.0) {
    mr31 = 1.0;
  } else if (mr31 < -1.0) {
    mr31 = -1.0;
  }
  rpy(1) = std::asin(mr31);
  rpy(0) = std::atan2(r32, r33);
  return rpy;
}

inline Eigen::Matrix3d EulerAnglesToDcm(const Eigen::Vector3d& rpy) {
  return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

std::string UrdfToSdf(const std::string& model_urdf_xml);

std::string GetRobotName(const std::string& model_sdf_xml);

}  // namespace gazebo_server

#endif  // GAZEBO_SERVER_HELPERS_H_
