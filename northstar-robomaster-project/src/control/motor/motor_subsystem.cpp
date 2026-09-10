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

void MotorSubsystem::setTargetRpm(float targetRpm) { this->targetRpm = targetRpm; }

void MotorSubsystem::refresh()
{
    // TODO(student): close the velocity loop. Roughly:
    //
    //   1. If the motor is not online (`motor.isMotorOnline()`), call zeroOutput() and
    //      return early. Skipping this lets the integral term wind up against a motor
    //      that is not listening, so the motor lurches when it reconnects. Note it is
    //      zeroOutput() and not stop() -- a momentary CAN dropout must not throw away
    //      the speed a command asked for.
    //   2. Compute the error: where we want to be (the `targetRpm` member, which
    //      setTargetRpm() filled in earlier this tick), minus where we are
    //      (getCurrentRpm()).
    //   3. Feed it to the PID: `velocityPid.runControllerDerivateError(error, dt)`.
    //      Pass `tap::Drivers::DT` as dt -- the scheduler calls refresh() exactly once
    //      per tick, and DT is how many milliseconds that tick is.
    //   4. Write the result out with `motor.setDesiredOutput(...)`.
    //
    // Until you do this, the motor will never move.
    motor.setDesiredOutput(0);
}

void MotorSubsystem::stop()
{
    // Clearing the target is what actually stops the motor -- refresh() runs after this
    // and would otherwise re-apply the old speed on the very same tick.
    targetRpm = 0.0f;
    zeroOutput();
}

void MotorSubsystem::zeroOutput()
{
    motor.setDesiredOutput(0);
    velocityPid.reset();
}

void MotorSubsystem::refreshSafeDisconnect() { stop(); }
}  // namespace src::motor
