/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

namespace src
{
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 *
 * Commands should never read `drivers->remote` directly. They ask this class
 * instead, so that "which stick does what" lives in exactly one file.
 */
class ControlOperatorInterface
{
public:
    /**
     * Inputs smaller than this are treated as zero. The sticks on the DR16 do not
     * reliably return exactly 0.0 when centered, and without a deadzone the motor
     * would creep whenever the remote is on.
     */
    static constexpr float STICK_DEADZONE = 0.01f;

    ControlOperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}

    /**
     * @return the operator's requested motor velocity, normalized to [-1, 1].
     *      Positive is "forward" (stick pushed up). Returns exactly 0 inside the
     *      deadzone.
     */
    mockable float getMotorVelocityInput();

private:
    tap::Drivers *drivers;
};
}  // namespace control

}  // namespace src

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_
