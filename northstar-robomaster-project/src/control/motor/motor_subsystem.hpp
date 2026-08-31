#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

namespace src::motor
{
/**
 * A Subsystem owning a single GM6020, with a velocity controller it exposes to Commands.
 *
 * A Subsystem is a piece of robot hardware. It owns the motor and is the only thing
 * allowed to talk to it. Exactly one Command may control a given Subsystem at a time --
 * the scheduler enforces that, which is what stops two Commands from fighting over the
 * same motor.
 *
 * The split to understand:
 *   - A Command decides *what we want* -- "spin at 200 RPM" -- and runs every tick in
 *     execute().
 *   - The Subsystem provides *how to get it* -- runVelocityPid() -- and owns the motor
 *     and the PID state.
 *
 * The control loop runs when a Command drives it, not on its own. That is deliberate,
 * and it is how the turret works in the real codebase: the turret subsystem holds the
 * motors, while the various controllers get run by whichever command is currently
 * aiming. It means you can later add a *position* controller, or one that holds a
 * heading from the IMU, as a sibling of runVelocityPid() and pick between them by
 * scheduling a different Command -- without the subsystem needing to know which mode it
 * is in.
 */
class MotorSubsystem : public tap::control::Subsystem
{
public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorId motorId,
        tap::can::CanBus canBus,
        bool isInverted,
        float gearRatio,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    /// Registers the motor with the CAN handler. Called once at startup.
    void initialize() override;

    /**
     * Drives the motor toward `targetRpm` by one PID step and writes the result out.
     *
     * Call this once per tick from a Command's execute(). Calling it at an irregular
     * rate will make the derivative and integral terms misbehave, since it assumes a
     * fixed timestep of tap::Drivers::DT.
     *
     * @param targetRpm the speed to aim for, in RPM.
     */
    mockable void runVelocityPid(float targetRpm);

    /**
     * Immediately commands zero output and clears the PID's accumulated state.
     *
     * Commands should call this in end(). Clearing the PID matters: without it, integral
     * wound up during the last run gets applied the instant the next command starts.
     */
    mockable void stop();

    /**
     * Called every tick by the scheduler, whether or not a command is running.
     *
     * Deliberately empty: control lives in runVelocityPid(), driven by a Command. It is
     * overridden here because Subsystem requires it, and to make the "where does the
     * control actually happen" question answerable by reading this file.
     */
    void refresh() override {}

    /// Called instead of refresh() when the remote disconnects. Must stop the motor.
    void refreshSafeDisconnect() override;

    /// @return the speed the motor is actually turning, in RPM.
    mockable float getCurrentRpm() const;

    /// @return the speed most recently asked for, in RPM.
    mockable float getTargetRpm() const { return targetRpm; }

    const char* getName() const override { return "GM6020 Motor"; }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    /// Test-only handle on the underlying mock motor, so tests can set expectations.
    Motor& getMotorForTest() { return motor; }
#endif

private:
    Motor motor;

    tap::algorithms::SmoothPid velocityPid;

    /// The speed most recently asked for, in RPM. Kept for introspection only.
    float targetRpm = 0.0f;
};  // class MotorSubsystem
}  // namespace src::motor
