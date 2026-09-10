/*
 * Tests for the GM6020 motor subsystem.
 *
 * Run them with:
 *     pipenv run scons run-tests profile=fast robot=STANDARD
 *
 * These are expected to FAIL until you have finished the TODOs in
 * src/control/motor/motor_subsystem.cpp. That is deliberate -- treat a passing
 * suite as the definition of "done" for the subsystem half of the exercise.
 *
 * Note on gains: these tests deliberately build their own PID config instead of
 * using VELOCITY_PID_CONFIG from standard_motor_constants.hpp, so that they test
 * *your control loop*, not *your tuning*. Tuning is checked on real hardware.
 */

#include <gtest/gtest.h>

#include "tap/drivers.hpp"

#include "control/motor/motor_subsystem.hpp"

using namespace src::motor;
using namespace testing;

namespace
{
/// A deliberately simple proportional-only config: output should be kp * error.
constexpr tap::algorithms::SmoothPidConfig TEST_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

class MotorSubsystemTest : public Test
{
protected:
    MotorSubsystemTest()
        : subsystem(
              &drivers,
              tap::motor::MOTOR5,
              tap::can::CanBus::CAN_BUS1,
              false,
              1.0f,
              TEST_PID_CONFIG)
    {
    }

    tap::Drivers drivers;
    MotorSubsystem subsystem;
};
}  // namespace

TEST_F(MotorSubsystemTest, target_rpm_starts_at_zero)
{
    EXPECT_FLOAT_EQ(0.0f, subsystem.getTargetRpm());
}

TEST_F(MotorSubsystemTest, set_target_rpm_records_the_target)
{
    subsystem.setTargetRpm(150.0f);

    EXPECT_FLOAT_EQ(150.0f, subsystem.getTargetRpm());
}

TEST_F(MotorSubsystemTest, set_target_rpm_alone_commands_nothing)
{
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    // Setting a target is a request, not a command. refresh() is what acts on it.
    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput).Times(0);

    subsystem.setTargetRpm(150.0f);
}

TEST_F(MotorSubsystemTest, stop_commands_zero_output)
{
    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(0));

    subsystem.stop();
}

TEST_F(MotorSubsystemTest, safe_disconnect_commands_zero_output)
{
    // Losing the remote must stop the motor.
    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(0));

    subsystem.refreshSafeDisconnect();
}

TEST_F(MotorSubsystemTest, offline_motor_commands_zero_output_without_losing_the_target)
{
    // With the motor unplugged we must write 0 rather than letting the integral
    // term wind up against a motor that is not listening.
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(false));

    subsystem.setTargetRpm(200.0f);

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(0));

    subsystem.refresh();

    // A momentary dropout zeroes the output, not what the command asked for -- so use
    // zeroOutput() here, not stop().
    EXPECT_FLOAT_EQ(200.0f, subsystem.getTargetRpm());
}

TEST_F(MotorSubsystemTest, positive_error_drives_positive_output)
{
    // The motor is online and stationary (no CAN feedback, so measured speed is 0),
    // and we ask it to spin forward. The PID should push the output positive.
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    subsystem.setTargetRpm(100.0f);

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(Gt(0)));

    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, negative_error_drives_negative_output)
{
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    subsystem.setTargetRpm(-100.0f);

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(Lt(0)));

    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, zero_target_produces_zero_output)
{
    // Stationary motor, asked for zero speed -> no error -> no output.
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(0));

    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, refresh_keeps_driving_toward_the_last_target)
{
    // The target latches: the subsystem keeps closing the loop on its own, without a
    // command re-stating what it wants every tick.
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    subsystem.setTargetRpm(100.0f);

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(Gt(0))).Times(2);

    subsystem.refresh();
    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, refresh_uses_the_most_recent_target)
{
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    subsystem.setTargetRpm(100.0f);
    subsystem.setTargetRpm(-100.0f);

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(Lt(0)));

    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, stop_clears_the_target_so_a_later_refresh_stays_at_zero)
{
    // This is the one that fails loudly if stop() only writes a zero without clearing
    // the target: refresh() runs after every command's end(), so it would put the motor
    // straight back to the speed we just stopped.
    ON_CALL(subsystem.getMotorForTest(), isMotorOnline).WillByDefault(Return(true));

    subsystem.setTargetRpm(200.0f);
    subsystem.stop();

    EXPECT_FLOAT_EQ(0.0f, subsystem.getTargetRpm());

    EXPECT_CALL(subsystem.getMotorForTest(), setDesiredOutput(0));

    subsystem.refresh();
}

TEST_F(MotorSubsystemTest, safe_disconnect_clears_the_target)
{
    // On the tick the remote comes back, refresh() runs before the default command is
    // re-scheduled. A target left over from before the disconnect would get one full
    // tick of output before anything corrected it.
    subsystem.setTargetRpm(200.0f);

    subsystem.refreshSafeDisconnect();

    EXPECT_FLOAT_EQ(0.0f, subsystem.getTargetRpm());
}
