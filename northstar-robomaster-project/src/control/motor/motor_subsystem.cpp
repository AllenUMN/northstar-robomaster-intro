#include "motor_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

namespace src::motor
{
MotorSubsystem::MotorSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorId motorId,
    tap::can::CanBus canBus,
    bool isInverted,
    float gearRatio,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : Subsystem(drivers),
      // `false` here is the currentControl flag: the GM6020 is driven by voltage, not
      // current, so it stays false.
      motor(drivers, motorId, canBus, isInverted, "GM6020", false, gearRatio),
      velocityPid(pidConfig)
{
}

void MotorSubsystem::initialize() { motor.initialize(); }

float MotorSubsystem::getCurrentRpm() const
{
    // TODO(student): return how fast the motor is actually spinning, in RPM.
    //
    // `motor.getEncoder()->getVelocity()` gives you the speed, but NOT in RPM -- read
    // the doc comment on EncoderInterface::getVelocity() in taproot to find out what
    // unit it actually returns, then convert.
    //
    // Getting this wrong is the most common way this exercise goes sideways: the PID
    // will still "work", it will just be regulating a number that is off by a constant
    // factor from what you think it is, and your gains will come out looking absurd.
    return 0.0f;
}

void MotorSubsystem::runVelocityPid(float targetRpm)
{
    this->targetRpm = targetRpm;

    // TODO(student): close the velocity loop. Roughly:
    //
    //   1. If the motor is not online (`motor.isMotorOnline()`), call stop() and return
    //      early. Skipping this lets the integral term wind up against a motor that is
    //      not listening, so the motor lurches when it reconnects.
    //   2. Compute the error: where we want to be, minus where we are (getCurrentRpm()).
    //   3. Feed it to the PID: `velocityPid.runControllerDerivateError(error, dt)`.
    //      Pass `tap::Drivers::DT` as dt -- this is called once per scheduler tick, and
    //      DT is how many milliseconds that tick is.
    //   4. Write the result out with `motor.setDesiredOutput(...)`.
    //
    // Until you do this, the motor will never move.
    motor.setDesiredOutput(0);
}

void MotorSubsystem::stop()
{
    motor.setDesiredOutput(0);
    velocityPid.reset();
}

void MotorSubsystem::refreshSafeDisconnect() { stop(); }
}  // namespace src::motor
