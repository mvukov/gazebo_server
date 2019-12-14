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
#ifndef GAZEBO_SERVER_LINK_H_
#define GAZEBO_SERVER_LINK_H_

#include <Eigen/Core>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo_server {

class GazeboServer;

/**
 * Wraps a subset of Gazebo's Link class methods in Eigen API.
 */
class Link {
 public:
  void GetWorldPose(Eigen::Vector3d* world_p_link,
                    Eigen::Matrix3d* world_r_link) const;
  Eigen::Vector3d GetWorldLinearVel() const;
  Eigen::Vector3d GetWorldAngularVel() const;
  Eigen::Vector3d GetWorldLinearAccel() const;
  Eigen::Vector3d GetWorldAngularAccel() const;

  Eigen::Vector3d GetRelativeLinearVel() const;
  Eigen::Vector3d GetRelativeLinearAccel() const;
  Eigen::Vector3d GetRelativeAngularVel() const;
  Eigen::Vector3d GetRelativeAngularAccel() const;

 protected:
  explicit Link(gazebo::physics::LinkPtr link) : link_(link) {}

 private:
  friend class GazeboServer;

  Link() = delete;

  gazebo::physics::LinkPtr link_;
};

}  // namespace gazebo_server

#endif  // GAZEBO_SERVER_LINK_H_
