/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "robot/control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace src
{
namespace control
{
float ControlOperatorInterface::getMotorVelocityInput()
{
    // taproot already normalizes remote channels to [-1, 1], so there is no
    // scaling to do here -- just reject the noise around center.
    float input = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);

    if (compareFloatClose(input, 0.0f, STICK_DEADZONE))
    {
        return 0.0f;
    }

    return input;
}

}  // namespace control

}  // namespace src
