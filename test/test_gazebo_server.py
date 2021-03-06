#!/usr/bin/env python3
# Copyright 2019 Milan Vukov. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests Python bindings for Gazebo server."""

import datetime
import os
import unittest

import numpy

from gazebo_server import py_gazebo_server


class ServerWithCallbacks:

  def __init__(self, package_path):
    config = py_gazebo_server.GazeboServer.Config()
    config.verbose = True
    config.world_path = os.path.join(package_path, 'test_data',
                                     'empty_test.world')

    model_sdf_path = os.path.join(package_path, 'test_data',
                                  'differential_drive', 'model.sdf')
    with open(model_sdf_path, 'r') as stream:
      config.model_sdf_xml = stream.read()

    self.server = py_gazebo_server.GazeboServer(config)
    if not self.server.start():
      raise RuntimeError('Failed to start the server!')

    self.num_on_world_update_begin_calls = 0
    self.num_on_world_update_end_calls = 0

  def on_world_update_begin(self):
    self.num_on_world_update_begin_calls += 1

  def on_world_update_end(self):
    self.num_on_world_update_end_calls += 1

  def run_for(self, num_steps):
    return self.server.run_for(num_steps, self.on_world_update_begin,
                               self.on_world_update_end)


class TestGazeboServer(unittest.TestCase):

  def setUp(self):
    unittest.TestCase.setUp(self)
    self.package_path = os.getcwd()

  def test_api(self):
    config = py_gazebo_server.GazeboServer.Config()
    config.verbose = True
    config.world_path = os.path.join(self.package_path, 'test_data',
                                     'empty_test.world')
    config.init_world_p_body = [1.2, 3.4, 0]
    config.init_world_rpy_body = [0, 0, 0]

    model_sdf_path = os.path.join(self.package_path, 'test_data',
                                  'differential_drive', 'model.sdf')
    with open(model_sdf_path, 'r') as stream:
      config.model_sdf_xml = stream.read()

    server = py_gazebo_server.GazeboServer(config)
    self.assertEqual(datetime.timedelta(0), server.simulation_time)
    self.assertTrue(server.start())
    self.assertTrue(server.step())
    self.assertEqual(datetime.timedelta(seconds=0.001), server.simulation_time)

    chassis = server.get_link('chassis')
    world_p_chassis, world_r_chassis = chassis.get_world_pose()
    numpy.testing.assert_almost_equal([1.2, 3.4, 0.1], world_p_chassis, 3e-6)
    numpy.testing.assert_almost_equal(
        py_gazebo_server.euler_angles_to_dcm(config.init_world_rpy_body),
        world_r_chassis, 2e-6)

    with self.assertRaises(RuntimeError):
      server.get_link('abcd')

    left_wheel_hinge = server.get_joint('left_wheel_hinge')
    right_wheel_hinge = server.get_joint('right_wheel_hinge')
    left_wheel_hinge.set_torque(1.0)
    right_wheel_hinge.set_torque(1.0)
    self.assertEqual(1.0, left_wheel_hinge.get_torque())
    self.assertEqual(1.0, right_wheel_hinge.get_torque())

    with self.assertRaises(RuntimeError):
      server.get_joint('efg')

  def test_run_for(self):
    test_server = ServerWithCallbacks(self.package_path)
    self.assertTrue(test_server.run_for(2))
    self.assertEqual(2, test_server.num_on_world_update_begin_calls)
    self.assertEqual(2, test_server.num_on_world_update_end_calls)


if __name__ == '__main__':
  unittest.main()
