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
#ifndef GAZEBO_SERVER_GAZEBO_SERVER_H_
#define GAZEBO_SERVER_GAZEBO_SERVER_H_

#include <functional>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include "gazebo_server/joint.h"
#include "gazebo_server/link.h"
#include "gazebo_server/time.h"

namespace gazebo_server {

class GazeboServer {
 public:
  struct Config {
    static constexpr double kAsFastAsPossible = 0.0;

    Config();

    std::string world_path = "worlds/empty.world";
    std::string model_sdf_xml;

    // Initial position, overrides the value in the model XML file.
    Eigen::Vector3d init_world_p_body;
    // Initial orientation expressed in Euler angles,
    // overrides the value in the model XML file.
    Eigen::Vector3d init_world_rpy_body;

    // Turn on to get verbose output from the server. Very useful for debugging.
    bool verbose = false;
    // The seed used for noise generation.
    int seed = 918273645;
    bool enable_physics_engine = true;

    // Overrides the XML value if >= 0.
    double real_time_update_rate = kAsFastAsPossible;

    // Returns true if configuration is valid, false otherwise.
    bool Validate() const;
  };

  using Callback = std::function<void()>;

  explicit GazeboServer(const Config& config) : config_(config) {}
  virtual ~GazeboServer();

  /**
   * Initializes the server.
   *
   * @returns True on success, false otherwise.
   */
  bool Start();

  /**
   * Executes a single simulation step.
   *
   * @returns True on success, false otherwise.
   */
  bool Step();

  /**
   * Executes a number of simulation steps.
   *
   * This is a blocking function call.
   *
   * @param num_steps The number of simulation steps to execute. Must be
   *                  larger than zero.
   * @param on_world_update_begin This is a mandatory callback called at
   *        the beginning of the beginning of the world update.
   * @param on_world_update_end This is an optional callback called at
   *        the end of the world update.
   *
   * @returns True on success, false otherwise. Fails if the simulator is
   *          not initialized, or if the arguments are invalid.
   */
  bool RunFor(int num_steps, Callback on_world_update_begin,
              Callback on_world_update_end);

  /**
   * Resets the simulator.
   *
   * True on success, false if the simulator is not initialized.
   */
  bool Reset();

  /**
   * Gets the simulation time.
   *
   * @returns The simulation time if the simulation is initialized, 0 otherwise.
   */
  SteadyTimestamp GetSimulationTime() const;

  /**
   * Gets the joint accessor.
   *
   * @param name The joint name.
   *
   * @returns The joint accessor on success,
   *          nullptr if the joint name is invalid.
   */
  std::unique_ptr<Joint> GetJoint(const std::string& name) const;

  /**
   * Gets the link accessor.
   *
   * @param name The link name.
   *
   * @returns The link accessor on success,
   *          nullptr if the link name is invalid.
   */
  std::unique_ptr<Link> GetLink(const std::string& name) const;

  const Config& config() const { return config_; }
  bool initialized() const { return initialized_; }
  const std::string& robot_name() const { return robot_name_; }

 protected:
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;

 private:
  bool IsReady() const;
  void ShutDown();

  const Config config_;
  bool initialized_ = false;
  std::string robot_name_;
};

}  // namespace gazebo_server

#endif  // GAZEBO_SERVER_GAZEBO_SERVER_H_
