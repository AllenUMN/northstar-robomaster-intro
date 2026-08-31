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
    // TODO(student): read the joystick and drive the motor toward that speed.
    //
    // `operatorInterface->getMotorVelocityInput()` gives you a number in [-1, 1].
    // `motor->runVelocityPid(...)` wants RPM. `MAX_MOTOR_RPM` (in
    // standard_motor_constants.hpp) is what full stick should correspond to.
    //
    // One line. Note that this is what actually steps the control loop -- the subsystem
    // does nothing on its own, so if this is empty the motor never moves.
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
