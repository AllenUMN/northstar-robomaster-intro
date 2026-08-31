#include "motor_velocity_command.hpp"

#include "robot/control_operator_interface.hpp"
#include "robot/standard/standard_motor_constants.hpp"

#include "motor_subsystem.hpp"

namespace src::motor
{
MotorVelocityCommand::MotorVelocityCommand(
    MotorSubsystem* motor,
    src::control::ControlOperatorInterface* operatorInterface)
    : motor(motor),
      operatorInterface(operatorInterface)
{
    // Tells the scheduler this command needs the motor. If another command that also
    // needs it gets scheduled, this one is interrupted rather than the two fighting.
    addSubsystemRequirement(motor);
}

void MotorVelocityCommand::execute()
{
    // Stick position in [-1, 1] scaled to the motor's usable speed range. This call is
    // what steps the control loop -- the subsystem does nothing on its own.
    motor->runVelocityPid(operatorInterface->getMotorVelocityInput() * MAX_MOTOR_RPM);
}

void MotorVelocityCommand::end([[maybe_unused]] bool interrupted)
{
    // Whatever the reason we stopped -- remote disconnected, another command took over
    // -- stop the motor rather than leaving it running at the last speed we commanded.
    // Nothing else will do this for us: the subsystem's refresh() is empty, so without
    // this the motor would happily keep spinning forever.
    motor->stop();
}
}  // namespace src::motor
