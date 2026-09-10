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
 * A Subsystem owning a single GM6020, closing a velocity loop on it every tick.
 *
 * A Subsystem is a piece of robot hardware. It owns the motor and is the only thing
 * allowed to talk to it. Exactly one Command may control a given Subsystem at a time --
 * the scheduler enforces that, which is what stops two Commands from fighting over the
 * same motor.
 *
 * The split to understand:
 *   - A Command decides *what we want* -- "spin at 200 RPM" -- and hands that over with
 *     setTargetRpm(). It never touches the motor or the PID.
 *   - The Subsystem decides *how to get it*: refresh() runs one PID step against the
 *     stored target and writes the output, every tick, whether or not a Command is
 *     scheduled.
 *
 * The target latches. Setting it commands nothing by itself, and not setting it commands
 * nothing new -- the loop keeps chasing the last value it was given. That is why a
 * Command must call stop() in end(): stop() clears the target, and nothing else does.
 *
 * The tradeoff, versus running the controller from the Command (which is how the turret
 * works in the real codebase). In favor: the scheduler calls refresh() exactly once per
 * tick, so the timestep handed to the PID really is tap::Drivers::DT, and a Command that
 * stops being scheduled cannot leave the motor holding a stale output. Against: this
 * Subsystem now has one control law baked into refresh(). Adding a *position* controller
 * later -- or one that holds a heading from the IMU -- means giving it a control *mode*,
 * switched on inside refresh(), rather than just adding a sibling method and picking
 * between them by scheduling a different Command. That mode is the price of the
 * Subsystem, rather than its caller, being responsible for what the motor is doing right
 * now.
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
     * Sets the speed the velocity loop should chase, in RPM.
     *
     * This commands the motor nothing by itself -- it only records the target. refresh()
     * is what acts on it, later in the same tick. The value latches: you stop the motor
     * with stop(), not by ceasing to call this.
     *
     * @param targetRpm the speed to aim for, in RPM.
     */
    mockable void setTargetRpm(float targetRpm);

    /**
     * Clears the target, commands zero output, and clears the PID's accumulated state.
     *
     * Commands must call this in end(). Clearing the target is the load-bearing part:
     * refresh() runs after every Command's execute(), so a stop() that only wrote a zero
     * would be undone by refresh() on the very same tick. Clearing the PID matters too --
     * integral wound up during the last run would otherwise land the instant the next
     * command starts.
     */
    mockable void stop();

    /**
     * Runs one step of the velocity loop and writes the result to the motor.
     *
     * Called by the scheduler every tick, after every scheduled Command's execute(), so
     * it always sees a target set this tick. It runs whether or not a Command is
     * scheduled, and it assumes the fixed timestep tap::Drivers::DT -- which holds,
     * because the scheduler calls it exactly once per tick.
     */
    void refresh() override;

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
    /// Commands zero and clears PID state, leaving the target alone.
    void zeroOutput();

    Motor motor;

    tap::algorithms::SmoothPid velocityPid;

    /// The speed the loop is chasing, in RPM. Read by refresh() every tick.
    float targetRpm = 0.0f;
};  // class MotorSubsystem
}  // namespace src::motor
