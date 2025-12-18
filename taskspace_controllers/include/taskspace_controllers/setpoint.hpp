// Copyright 2024 Alex Arbogast
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

#pragma once

#include <taskspace_controllers/utility.hpp>
#include <kdl/frames.hpp>

namespace taskspace_controllers
{

struct PoseTwistSetpoint
{
  PoseTwistSetpoint(const KDL::Frame& pose = KDL::Frame::Identity(),
                    const ctrl::Vector6D& twist = ctrl::Vector6D::Zero())
    : pose(pose), twist(twist)
  {
  }

  KDL::Frame pose;
  ctrl::Vector6D twist;
};

}  // namespace taskspace_controllers
