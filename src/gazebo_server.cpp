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
#include "gazebo_server/gazebo_server.h"

#include <thread>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <ode/ode.h>

#include "gazebo_server/helpers.h"

// TODO(mvukov) In principle, we could route those as Gazebo warnings.
extern "C" void SilenceOdeMessages(int, const char*, va_list) {}

namespace gazebo_server {

GazeboServer::Config::Config() {
  init_world_p_body.setZero();
  init_world_rpy_body.setZero();
}

bool GazeboServer::Config::Validate() const {
  for (const auto& path : media_paths) {
    if (path.empty()) {
      std::cerr << "Got an empty media path!" << std::endl;
      return false;
    }
  }
  for (const auto& path : model_paths) {
    if (path.empty()) {
      std::cerr << "Got an empty model path!" << std::endl;
      return false;
    }
  }
  if (world_path.empty()) {
    std::cerr << "Got an empty world configuration file path!" << std::endl;
    return false;
  }
  if (model_sdf_xml.empty()) {
    std::cerr << "Got an empty model XML file!" << std::endl;
    return false;
  }
  return true;
}

GazeboServer::~GazeboServer() { ShutDown(); }

bool GazeboServer::Start() {
  if (!config_.Validate()) {
    return false;
  }

  if (gazebo::physics::has_world()) {
    gzerr << "Another server has been set up in the same process already!"
          << std::endl;
    return false;
  }

  robot_name_ = GetRobotName(config_.model_sdf_xml);
  if (robot_name_.empty()) {
    return false;
  }

  std::vector<std::string> gazebo_args;
  if (config_.verbose) {
    gazebo::printVersion();
    gazebo::common::Console::SetQuiet(false);
    gazebo_args.push_back("--verbose");
  } else {
    gazebo::common::Console::SetQuiet(true);
  }

  if (!gazebo::setupServer(gazebo_args)) {
    gzerr << "Failed to set up server!" << std::endl;
    ShutDown();
    return false;
  }

  for (const auto& path : config_.media_paths) {
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path);
  }
  for (const auto& path : config_.model_paths) {
    gazebo::common::SystemPaths::Instance()->AddModelPaths(path);
  }

  gzmsg << "Loading world..." << std::endl;
  world_ = gazebo::loadWorld(config_.world_path);
  if (world_ == nullptr) {
    gzerr << "Failed to fetch world!" << std::endl;
    ShutDown();
    return false;
  }

  gzmsg << "Loading model..." << std::endl;
  world_->InsertModelString(config_.model_sdf_xml);

  static constexpr int kNumAttempts = 500;
  static constexpr int kTimeoutMsec = 10;
  int attempt = 0;
  for (; attempt < kNumAttempts; ++attempt) {
    gazebo::runWorld(world_, 1);
    model_ = world_->ModelByName(robot_name_);
    if (model_ != nullptr) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kTimeoutMsec));
  }
  if (attempt == kNumAttempts) {
    gzerr << "Failed to fetch robot model with name: " << robot_name_
          << std::endl;
    ShutDown();
    return false;
  }
  assert(model_ != nullptr);
  gzmsg << "Robot model became available after <= "
        << (attempt + 1) * kTimeoutMsec << " [ms]" << std::endl;

  initialized_ = true;
  Reset();

  if (!config_.verbose) {
    auto physics_engine = world_->Physics();
    if (physics_engine->GetType() == "ode") {
      dSetMessageHandler(&SilenceOdeMessages);
    }
  }

  std::stringstream sstream;
  sstream << "Model links: ";
  for (auto link : model_->GetLinks()) {
    sstream << link->GetName() << " ";
  }
  sstream << ". Model joints: ";
  for (auto joint : model_->GetJoints()) {
    sstream << joint->GetName() << " ";
  }
  gzmsg << sstream.str() << std::endl;

  return true;
}

bool GazeboServer::Step() {
  if (!IsReady()) {
    return false;
  }
  gazebo::runWorld(world_, 1);
  return true;
}

bool GazeboServer::RunFor(int num_steps, Callback on_world_update_begin,
                          Callback on_world_update_end) {
  if (!IsReady()) {
    return false;
  }
  if (num_steps < 1) {
    gzerr << "The number of requested steps must be larger than zero!"
          << std::endl;
    return false;
  }
  if (!on_world_update_begin) {
    gzerr << "on_world_update_begin callback must be defined!" << std::endl;
    return false;
  }

  gazebo::event::ConnectionPtr world_begin;
  gazebo::event::ConnectionPtr world_end;

  world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
      [&on_world_update_begin](const gazebo::common::UpdateInfo&) {
        on_world_update_begin();
      });

  if (on_world_update_end) {
    world_end =
        gazebo::event::Events::ConnectWorldUpdateEnd(on_world_update_end);
  }

  gazebo::runWorld(world_, num_steps);

  return true;
}

bool GazeboServer::Reset() {
  if (!IsReady()) {
    return false;
  }

  const Eigen::Vector3d& world_p_body = config_.init_world_p_body;
  const Eigen::Vector3d& world_rpy_body = config_.init_world_rpy_body;
  ignition::math::Pose3d initial_pose(world_p_body.x(), world_p_body.y(),
                                      world_p_body.z(), world_rpy_body.x(),
                                      world_rpy_body.y(), world_rpy_body.z());

  model_->SetInitialRelativePose(initial_pose);
  model_->SetRelativePose(initial_pose);
  ignition::math::Rand::Seed(config_.seed);

  world_->Reset();

  if (config_.real_time_update_rate >= 0) {
    auto physics_engine = world_->Physics();
    physics_engine->SetRealTimeUpdateRate(config_.real_time_update_rate);
  }

  return true;
}

SteadyTimestamp GazeboServer::GetSimulationTime() const {
  if (!initialized_) {
    return GetTimestamp(0);
  }
  const auto gazebo_timestamp = world_->SimTime();
  return GetTimestamp(gazebo_timestamp.sec, gazebo_timestamp.nsec);
}

bool GazeboServer::IsReady() const {
  const auto ready = initialized_ && !world_->Running();
  if (!ready) {
    gzerr << "The server is not initialized and/or is already running!"
          << std::endl;
  }
  return ready;
}

void GazeboServer::ShutDown() {
  gazebo::event::Events::stop();
  if ((world_ != nullptr || model_ != nullptr) && !gazebo::shutdown()) {
    std::cerr << "Failed to shut down the server!" << std::endl;
  }
}

std::unique_ptr<Link> GazeboServer::GetLink(const std::string& name) const {
  if (!initialized_) return nullptr;
  auto link = model_->GetLink(name);
  if (link == nullptr) {
    gzerr << "Failed to find link " << name << "!" << std::endl;
    return nullptr;
  }
  return std::unique_ptr<Link>(new Link(link));
}

std::unique_ptr<Joint> GazeboServer::GetJoint(const std::string& name) const {
  if (!initialized_) return nullptr;
  auto joint = model_->GetJoint(name);
  if (joint == nullptr) {
    gzerr << "Failed to find joint: " << name << "!" << std::endl;
    return nullptr;
  }
  return std::unique_ptr<Joint>(new Joint(joint));
}

}  // namespace gazebo_server
