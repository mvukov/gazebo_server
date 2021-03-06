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
#ifndef GAZEBO_SERVER_TEST_ENTRY_POINT_H_
#define GAZEBO_SERVER_TEST_ENTRY_POINT_H_

#include <gtest/gtest.h>

#define TEST_ENTRY_POINT                                    \
  int main(int argc, char* argv[]) {                        \
    ::testing::InitGoogleTest(&argc, argv);                 \
    ::testing::FLAGS_gtest_death_test_style = "threadsafe"; \
    const int success = RUN_ALL_TESTS();                    \
    return success;                                         \
  }

#endif  // GAZEBO_SERVER_TEST_ENTRY_POINT_H_
