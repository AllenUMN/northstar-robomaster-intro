#ifndef STANDARD_MOTOR_CONSTANTS_HPP_
#define STANDARD_MOTOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::motor
{
/**
 * Everything about *this particular robot's* motor lives here: which bus it is on,
 * which ID it answers to, and how the velocity loop is tuned. The subsystem itself
 * (src/control/motor/) stays generic and takes all of this as constructor arguments.
 */

static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

/**
 * A GM6020's CAN ID is set by the dial on the back of the motor. Dial position 1
 * transmits on 0x205, which taproot calls MOTOR5. Dial position 2 is 0x206 (MOTOR6),
 * and so on. If your motor does not respond, this is the first thing to check.
 */
static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR5;

/// Set true to flip which direction counts as positive.
static constexpr bool MOTOR_INVERTED = false;

/**
 * The GM6020 is a direct-drive motor -- there is no gearbox between the rotor and the
 * output shaft -- so its gear ratio is 1.
 */
static constexpr float MOTOR_GEAR_RATIO = 1.0f;

/// Roughly the GM6020's free-running speed. Full stick maps to this.
static constexpr float MAX_MOTOR_RPM = 320.0f;

/**
 * Gains for the velocity loop.
 *
 * TODO(student): these are all zero, so the motor will not move no matter what you do
 * to the stick. Tune them once the rest of the code is written:
 *   1. Raise kp until the motor reaches roughly the speed you asked for.
 *   2. Add kd if it oscillates or overshoots.
 *   3. Add ki last, and only if it consistently settles short of the target.
 *
 * maxOutput is already correct and should be left alone: the GM6020 is commanded in
 * *voltage*, and 25000 (MAX_OUTPUT_GM6020) is full scale. Using the C620 value here
 * instead would silently limit you to about 65% output.
 */
static constexpr tap::algorithms::SmoothPidConfig VELOCITY_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace src::motor

#endif  // STANDARD_MOTOR_CONSTANTS_HPP_
