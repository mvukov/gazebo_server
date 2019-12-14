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
#include "gazebo_server/link.h"

#include <Eigen/Geometry>
#include <gazebo/physics/Link.hh>

namespace gazebo_server {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

void Link::GetWorldPose(Vector3d* world_p_link, Matrix3d* world_r_link) const {
  assert(world_p_link != nullptr);
  assert(world_r_link != nullptr);

  const auto world_t_link = link_->WorldPose();
  *world_p_link = {world_t_link.Pos().X(), world_t_link.Pos().Y(),
                   world_t_link.Pos().Z()};
  *world_r_link = Quaterniond(world_t_link.Rot().W(), world_t_link.Rot().X(),
                              world_t_link.Rot().Y(), world_t_link.Rot().Z())
                      .toRotationMatrix();
}
Vector3d Link::GetWorldLinearVel() const {
  const auto ret = link_->WorldLinearVel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetWorldAngularVel() const {
  const auto ret = link_->WorldAngularVel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetWorldLinearAccel() const {
  const auto ret = link_->WorldLinearAccel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetWorldAngularAccel() const {
  const auto ret = link_->WorldAngularAccel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}

Vector3d Link::GetRelativeLinearVel() const {
  const auto ret = link_->RelativeLinearVel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetRelativeLinearAccel() const {
  const auto ret = link_->RelativeLinearAccel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetRelativeAngularVel() const {
  const auto ret = link_->RelativeAngularVel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}
Vector3d Link::GetRelativeAngularAccel() const {
  const auto ret = link_->RelativeAngularAccel();
  return Vector3d(ret.X(), ret.Y(), ret.Z());
}

}  // namespace gazebo_server
