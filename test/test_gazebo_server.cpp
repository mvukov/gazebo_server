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
#include <fstream>
#include <memory>
#include <string>

#include "gazebo_server/gazebo_server.h"
#include "gazebo_server/helpers.h"

#include "./test_entry_point.h"

namespace gazebo_server {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class TestGazeboServerConfig : public ::testing::Test {
 protected:
  GazeboServer::Config config_;
};

TEST_F(TestGazeboServerConfig, Success) {
  EXPECT_FALSE(config_.Validate());
  config_.model_sdf_xml = "foo";
  EXPECT_TRUE(config_.Validate());
  EXPECT_EQ(Vector3d::Zero(), config_.init_world_p_body);
  EXPECT_EQ(Vector3d::Zero(), config_.init_world_rpy_body);
}

TEST_F(TestGazeboServerConfig, WorldEmptyFailure) {
  config_.world_path = "";
  EXPECT_FALSE(config_.Validate());
}

TEST_F(TestGazeboServerConfig, ServerInitFailure) {
  GazeboServer server(config_);
  EXPECT_FALSE(server.Start());
}

class TestGazeboServer : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    config_.verbose = true;

    const std::string test_data_path(TEST_DATA_PATH);
    ASSERT_FALSE(test_data_path.empty());

    config_.world_path = test_data_path + "/empty_test.world";

    {
      const std::string model_path =
          test_data_path + "/differential_drive/model.sdf";
      std::ifstream stream(model_path.c_str());
      std::stringstream sstream;
      sstream << stream.rdbuf();
      config_.model_sdf_xml = sstream.str();
    }

    config_.init_world_p_body = {1, 2, 0};
    config_.init_world_rpy_body = {0, 0, 0};

    server_ = std::make_unique<GazeboServer>(config_);
    ASSERT_NE(server_, nullptr);

    // Test uninitialized state of the server.
    ASSERT_FALSE(server_->Step());
    ASSERT_FALSE(server_->RunFor(
        2, []() {}, GazeboServer::Callback()));
    ASSERT_FALSE(server_->Reset());
    ASSERT_EQ(GetTimestamp(0), server_->GetSimulationTime());
    ASSERT_EQ(nullptr, server_->GetJoint("right_wheel_hinge"));
    ASSERT_EQ(nullptr, server_->GetLink("right_wheel"));
    ASSERT_FALSE(server_->initialized());
    ASSERT_TRUE(server_->robot_name().empty());

    ASSERT_TRUE(server_->Start());
  }

  static void SetUpTestCase() { SetUpTestSuite(); }

  static void TearDownTestSuite() { server_.reset(); }

  static void TearDownTestCase() { TearDownTestSuite(); }

 protected:
  void SetUp() override { ASSERT_TRUE(server_->Reset()); }

  static GazeboServer::Config config_;
  static std::unique_ptr<GazeboServer> server_;
};

GazeboServer::Config TestGazeboServer::config_;
std::unique_ptr<GazeboServer> TestGazeboServer::server_ = nullptr;

TEST_F(TestGazeboServer, Initialized) {
  ASSERT_TRUE(server_->initialized());
  ASSERT_EQ("differential_drive", server_->robot_name());
  ASSERT_EQ(GetTimestamp(0), server_->GetSimulationTime());
  ASSERT_TRUE(server_->Step());
  EXPECT_GT(server_->GetSimulationTime(), GetTimestamp(0));

  int num_on_world_update_begin_calls = 0;
  int num_on_world_update_end_calls = 0;

  static constexpr int kStepNsec = 1000000;  // 1ms.

  ASSERT_TRUE(server_->RunFor(
      2,
      [&num_on_world_update_begin_calls]() {
        ++num_on_world_update_begin_calls;
        EXPECT_EQ(
            GetTimestamp(0, kStepNsec * (num_on_world_update_begin_calls + 1)),
            server_->GetSimulationTime());
      },
      [&num_on_world_update_end_calls]() {
        ++num_on_world_update_end_calls;
        EXPECT_EQ(
            GetTimestamp(0, kStepNsec * (num_on_world_update_end_calls + 1)),
            server_->GetSimulationTime());
      }));
  EXPECT_EQ(2, num_on_world_update_begin_calls);
  EXPECT_EQ(2, num_on_world_update_end_calls);

  ASSERT_EQ(nullptr, server_->GetJoint("foo"));
  ASSERT_EQ(nullptr, server_->GetLink("bar"));

  ASSERT_NE(nullptr, server_->GetJoint("left_wheel_hinge"));
  ASSERT_NE(nullptr, server_->GetLink("left_wheel"));
}

TEST_F(TestGazeboServer, SecondServerInstanceFailure) {
  GazeboServer server2(config_);
  EXPECT_FALSE(server2.Start());
}

TEST_F(TestGazeboServer, NoMovement) {
  auto left_wheel_hinge = server_->GetJoint("left_wheel_hinge");
  auto right_wheel_hinge = server_->GetJoint("right_wheel_hinge");
  auto chassis = server_->GetLink("chassis");

  ASSERT_NE(nullptr, left_wheel_hinge);
  ASSERT_NE(nullptr, right_wheel_hinge);
  ASSERT_NE(nullptr, chassis);

  ASSERT_EQ(0, left_wheel_hinge->GetTorque());
  ASSERT_EQ(0, right_wheel_hinge->GetTorque());

  const Vector3d world_p_chassis_0{1, 2, 0.1};
  const Matrix3d world_r_chassis_0 = Matrix3d::Identity();

  Vector3d world_p_chassis_1;
  Matrix3d world_r_chassis_1;
  chassis->GetWorldPose(&world_p_chassis_1, &world_r_chassis_1);

  EXPECT_EQ(world_p_chassis_0, world_p_chassis_1);
  EXPECT_EQ(world_r_chassis_0, world_r_chassis_1);

  Vector3d world_p_chassis_2;
  Matrix3d world_r_chassis_2;
  ASSERT_TRUE(server_->Step());
  chassis->GetWorldPose(&world_p_chassis_2, &world_r_chassis_2);

  const auto p_error_2 =
      (world_p_chassis_1 - world_p_chassis_2).cwiseAbs().maxCoeff();
  const auto angle_error_2 =
      DcmToEulerAngles(world_r_chassis_1 * world_r_chassis_2.transpose())
          .cwiseAbs()
          .maxCoeff();

  EXPECT_LE(p_error_2, 3e-6);
  EXPECT_LE(angle_error_2, 2e-6);
}

TEST_F(TestGazeboServer, Repeatability) {
  auto left_wheel_hinge = server_->GetJoint("left_wheel_hinge");
  auto right_wheel_hinge = server_->GetJoint("right_wheel_hinge");
  auto chassis = server_->GetLink("chassis");

  static constexpr int kNumSteps = 100;
  static constexpr double kTorque = 2.0;

  std::vector<Vector3d> world_p_chassis_1;
  std::vector<Matrix3d> world_r_chassis_1;
  for (int step = 0; step < kNumSteps; ++step) {
    left_wheel_hinge->SetTorque(kTorque);
    right_wheel_hinge->SetTorque(kTorque);

    Vector3d world_p_chassis;
    Matrix3d world_r_chassis;
    chassis->GetWorldPose(&world_p_chassis, &world_r_chassis);
    world_p_chassis_1.push_back(world_p_chassis);
    world_r_chassis_1.push_back(world_r_chassis);
    ASSERT_TRUE(server_->Step());
  }
  ASSERT_TRUE(server_->Reset());
  std::vector<Vector3d> world_p_chassis_2;
  std::vector<Matrix3d> world_r_chassis_2;
  for (int step = 0; step < kNumSteps; ++step) {
    left_wheel_hinge->SetTorque(kTorque);
    right_wheel_hinge->SetTorque(kTorque);

    Vector3d world_p_chassis;
    Matrix3d world_r_chassis;
    chassis->GetWorldPose(&world_p_chassis, &world_r_chassis);
    world_p_chassis_2.push_back(world_p_chassis);
    world_r_chassis_2.push_back(world_r_chassis);
    ASSERT_TRUE(server_->Step());
  }

  for (int step = 0; step < kNumSteps; ++step) {
    EXPECT_EQ(world_p_chassis_1.at(step), world_p_chassis_2.at(step))
        << world_p_chassis_1.at(step).transpose()
        << ", 2: " << world_p_chassis_2.at(step).transpose();
    EXPECT_EQ(world_r_chassis_1.at(step), world_r_chassis_2.at(step));
  }

  static constexpr int kStepNsec = 1000000;  // 1ms.
  EXPECT_EQ(GetTimestamp(0, kNumSteps * kStepNsec),
            server_->GetSimulationTime());
}

}  // namespace gazebo_server

TEST_ENTRY_POINT
