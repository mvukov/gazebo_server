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
#include "gazebo_server/joint.h"

#include <gazebo/physics/Joint.hh>

namespace gazebo_server {
namespace {

constexpr int kAxis = 0;

}  // namespace

void Joint::SetTorque(double torque) { joint_->SetForce(kAxis, torque); }

double Joint::GetTorque() const { return joint_->GetForce(kAxis); }

double Joint::GetVelocity() const { return joint_->GetVelocity(kAxis); }

double Joint::GetPosition() const { return joint_->Position(kAxis); }

}  // namespace gazebo_server
