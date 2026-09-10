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
    // TODO(student): read the joystick and hand the subsystem that target speed.
    //
    // `operatorInterface->getMotorVelocityInput()` gives you a number in [-1, 1].
    // `motor->setTargetRpm(...)` wants RPM. `MAX_MOTOR_RPM` (in
    // standard_motor_constants.hpp) is what full stick should correspond to.
    //
    // One line. Note this only records a target -- the subsystem's refresh() is what
    // steps the control loop, later in this same tick. If this is empty the motor sits
    // at 0 RPM under active control rather than doing nothing.
}

void MotorVelocityCommand::end([[maybe_unused]] bool interrupted)
{
    // Whatever the reason we stopped -- remote disconnected, another command took over
    // -- clear the target. The subsystem's refresh() keeps running after we are gone and
    // would happily hold the last speed we asked for forever; stop() is the only thing
    // that clears it.
    motor->stop();
}
}  // namespace src::motor
