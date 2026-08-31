#pragma once

#include "tap/control/command.hpp"

namespace src
{
namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::motor
{
class MotorSubsystem;

/**
 * Drives the motor from the operator's joystick.
 *
 * This is a Command: it decides *what the robot should be doing right now*. It does not
 * touch the motor or the PID -- it only translates operator input into a target speed
 * and hands that to the Subsystem.
 *
 * Lifecycle, all driven by the command scheduler:
 *   initialize()  once, when the command is scheduled
 *   execute()     every tick, for as long as it is scheduled
 *   isFinished()  every tick; returning true ends the command
 *   end()         once, when it stops (either because it finished, or because
 *                 something else took the subsystem away -- `interrupted` says which)
 *
 * Because this is registered as the subsystem's *default* command, it runs whenever
 * nothing else has claimed the motor, and it never finishes on its own.
 */
class MotorVelocityCommand : public tap::control::Command
{
public:
    MotorVelocityCommand(
        MotorSubsystem *motor,
        src::control::ControlOperatorInterface *operatorInterface);

    const char *getName() const override { return "Motor velocity"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

private:
    src::motor::MotorSubsystem *motor;

    src::control::ControlOperatorInterface *operatorInterface;
};
}  // namespace src::motor
