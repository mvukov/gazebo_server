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
#ifndef GAZEBO_SERVER_TIME_H_
#define GAZEBO_SERVER_TIME_H_

#include <chrono>

namespace gazebo_server {

using SteadyClock = std::chrono::steady_clock;
using SteadyTimestamp = SteadyClock::time_point;

constexpr SteadyTimestamp GetTimestamp(int seconds, int nanoseconds) {
  return SteadyTimestamp(SteadyTimestamp::duration(
      std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanoseconds)));
}

constexpr SteadyTimestamp GetTimestamp(int seconds) {
  return GetTimestamp(seconds, 0);
}

}  // namespace gazebo_server

#endif  // GAZEBO_SERVER_TIME_H_
