#include "motor_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "modm/math/geometry/angle.hpp"

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
    // getVelocity() reports post-gearbox radians per second. Radians -> revolutions is
    // a divide by 2*pi, seconds -> minutes is a multiply by 60.
    //
    // No gear ratio appears here on purpose: the encoder already accounts for it (that
    // is what "post-gearbox" means), and the GM6020 is direct-drive anyway. Dividing by
    // the gear ratio a second time is the classic bug in this function.
    return motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI;
}

void MotorSubsystem::runVelocityPid(float targetRpm)
{
    this->targetRpm = targetRpm;

    // Nothing useful to do against a motor that is not answering, and running the PID
    // anyway would wind the integral term up while the output goes nowhere -- so the
    // motor would lurch the instant it reconnected.
    if (!motor.isMotorOnline())
    {
        stop();
        return;
    }

    const float error = targetRpm - getCurrentRpm();

    velocityPid.runControllerDerivateError(error, tap::Drivers::DT);

    motor.setDesiredOutput(velocityPid.getOutput());
}

void MotorSubsystem::stop()
{
    motor.setDesiredOutput(0);
    velocityPid.reset();
}

void MotorSubsystem::refreshSafeDisconnect() { stop(); }
}  // namespace src::motor
