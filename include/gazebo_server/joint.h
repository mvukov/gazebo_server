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
#ifndef GAZEBO_SERVER_JOINT_H_
#define GAZEBO_SERVER_JOINT_H_

#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo_server {

class GazeboServer;

/**
 * Wraps a subset of Gazebo's Joint class methods.
 *
 * The API acts on the 0-th axis of the joint.
 */
class Joint {
 public:
  void SetTorque(double torque);
  double GetTorque() const;

  double GetVelocity() const;
  double GetPosition() const;

 protected:
  explicit Joint(gazebo::physics::JointPtr joint) : joint_(joint) {}

 private:
  friend class GazeboServer;

  Joint() = delete;

  gazebo::physics::JointPtr joint_;
};

}  // namespace gazebo_server

#endif  // GAZEBO_SERVER_JOINT_H_
