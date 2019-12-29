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
#include <tuple>

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gazebo_server/gazebo_server.h"
#include "gazebo_server/helpers.h"
#include "gazebo_server/joint.h"
#include "gazebo_server/link.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace gazebo_server {

PYBIND11_MODULE(py_gazebo_server, m) {
  m.doc() = "Gazebo server Python bindings";

  py::class_<Joint>(m, "Joint")
      .def("get_torque", &Joint::GetTorque)
      .def("set_torque", &Joint::SetTorque, "torque"_a)
      .def("get_velocity", &Joint::GetVelocity)
      .def("get_position", &Joint::GetPosition);

  py::class_<Link>(m, "Link")
      .def(
          "get_world_pose",
          [](const Link& self) -> std::tuple<Eigen::Vector3d, Eigen::Matrix3d> {
            Eigen::Vector3d world_p_link;
            Eigen::Matrix3d world_r_link;
            self.GetWorldPose(&world_p_link, &world_r_link);
            return std::make_tuple(world_p_link, world_r_link);
          },
          "Returns <world_p_link, world_r_link>.")
      .def("get_world_linear_vel", &Link::GetWorldLinearVel)
      .def("get_world_angular_vel", &Link::GetWorldAngularVel)
      .def("get_world_linear_accel", &Link::GetWorldLinearAccel)
      .def("get_world_angular_accel", &Link::GetWorldAngularAccel)
      .def("get_relative_linear_vel", &Link::GetRelativeLinearVel)
      .def("get_relative_linear_accel", &Link::GetRelativeLinearAccel)
      .def("get_relative_angular_vel", &Link::GetRelativeAngularVel)
      .def("get_relative_angular_accel", &Link::GetRelativeAngularAccel);

  py::class_<GazeboServer> server(m, "GazeboServer");

  py::class_<GazeboServer::Config>(server, "Config")
      .def(py::init<>())
      .def("validate", &GazeboServer::Config::Validate)
      .def_readwrite("media_paths", &GazeboServer::Config::media_paths)
      .def_readwrite("model_paths", &GazeboServer::Config::model_paths)
      .def_readwrite("world_path", &GazeboServer::Config::world_path)
      .def_readwrite("model_sdf_xml", &GazeboServer::Config::model_sdf_xml)
      .def_readwrite("init_world_p_body",
                     &GazeboServer::Config::init_world_p_body)
      .def_readwrite("init_world_rpy_body",
                     &GazeboServer::Config::init_world_rpy_body)
      .def_readwrite("verbose", &GazeboServer::Config::verbose)
      .def_readwrite("seed", &GazeboServer::Config::seed)
      .def_readwrite("enable_physics_engine",
                     &GazeboServer::Config::enable_physics_engine)
      .def_readwrite("real_time_update_rate",
                     &GazeboServer::Config::real_time_update_rate);

  server.def(py::init<const GazeboServer::Config&>())
      .def("start", &GazeboServer::Start)
      .def("step", &GazeboServer::Step)

      // TODO(mvukov) Investigate further why this does not work.
      // On macOS I get segfault at exit, RenderEngine/WindowManager related.
      // The server exits cleanly, but some global Gazebo objects complain.
      // On Ubuntu 18.04 (CI) the Python test does not exit on it's own.
      // I need to do CTRL+C.
      // .def("run_for", &GazeboServer::RunFor, "num_steps"_a,
      //      "on_world_update_begin"_a, "on_world_update_end"_a)

      .def("reset", &GazeboServer::Reset)
      .def_property_readonly("simulation_time",
                             &GazeboServer::GetSimulationTime)

      .def(
          "get_joint",
          [](const GazeboServer& self, const std::string& name) {
            auto joint = self.GetJoint(name);
            if (joint == nullptr) {
              throw std::runtime_error("Failed to get joint!");
            }
            return joint;
          },
          "name"_a)

      .def(
          "get_link",
          [](const GazeboServer& self, const std::string& name) {
            auto link = self.GetLink(name);
            if (link == nullptr) {
              throw std::runtime_error("Failed to get link!");
            }
            return link;
          },
          "name"_a);

  m.def("urdf_to_sdf", &UrdfToSdf, "model_urdf_xml"_a);
  m.def("dcm_to_euler_angles", &DcmToEulerAngles, "global_r_local"_a);
  m.def("euler_angles_to_dcm", &EulerAnglesToDcm, "euler_angles"_a);
}

}  // namespace gazebo_server
