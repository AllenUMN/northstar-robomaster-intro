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
 * !! THESE ARE A STARTING POINT, NOT A TUNED RESULT. !!
 *
 * They were reasoned about, not measured -- nobody has run them against a real motor.
 * Expect to adjust them. They are sized so that the first run is sluggish rather than
 * violent, which is the safe direction to be wrong in.
 *
 * The reasoning, so you can re-derive it if your setup differs: error is in RPM and
 * output is in millivolts, full scale 25000. kp = 200 means a 125 RPM error saturates
 * the output, so anything short of a large step responds proportionally. ki is kept
 * small because runControllerDerivateError() is passed dt in *milliseconds*, so the
 * integral accumulates in steps of ki * error * 2 every tick and winds up faster than
 * you would expect coming from a seconds-based loop. kd is 0 because a direct-drive
 * velocity loop with a Kalman-filtered derivative rarely needs it -- add it only if you
 * see oscillation.
 *
 * To tune:
 *   1. Set ki and kd to 0. Raise kp until the motor reaches roughly the speed you asked
 *      for and responds crisply, backing off if it buzzes or oscillates.
 *   2. Add ki only if it consistently settles short of the target under load.
 *   3. Add kd only to damp overshoot.
 *
 * maxOutput is already correct and should be left alone: the GM6020 is commanded in
 * *voltage*, and 25000 (MAX_OUTPUT_GM6020) is full scale. Using the C620 value here
 * instead would silently limit you to about 65% output.
 */
static constexpr tap::algorithms::SmoothPidConfig VELOCITY_PID_CONFIG = {
    .kp = 200.0f,
    .ki = 0.5f,
    .kd = 0.0f,
    .maxICumulative = 5000.0f,
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
