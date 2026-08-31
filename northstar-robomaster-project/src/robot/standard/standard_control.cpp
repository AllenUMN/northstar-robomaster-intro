#ifdef TARGET_STANDARD

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/motor/motor_subsystem.hpp"
#include "control/motor/motor_velocity_command.hpp"
#include "control/safe_disconnect.hpp"
#include "robot/standard/standard_drivers.hpp"
#include "robot/standard/standard_motor_constants.hpp"

#include "drivers_singleton.hpp"

using namespace tap::control;
using namespace src::control;
using namespace src::standard;

driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
/*
 * This file is where the robot gets assembled. Everything below is constructed once, at
 * startup, and lives for the whole run -- there is no dynamic allocation.
 *
 * The pattern, which every robot in the real codebase follows:
 *   1. Declare the subsystems (the hardware).
 *   2. Declare the commands (the behaviors).
 *   3. Register the subsystems with the scheduler.
 *   4. Give each subsystem a default command.
 *   5. Map any remaining commands to buttons/switches.
 */

// ---------------------------------------------------------------------------
// Subsystems
// ---------------------------------------------------------------------------

src::motor::MotorSubsystem motorSubsystem(
    drivers(),
    src::motor::MOTOR_ID,
    src::motor::MOTOR_CAN_BUS,
    src::motor::MOTOR_INVERTED,
    src::motor::MOTOR_GEAR_RATIO,
    src::motor::VELOCITY_PID_CONFIG);

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

src::motor::MotorVelocityCommand motorVelocityCommand(
    &motorSubsystem,
    &drivers()->controlOperatorInterface);

// ---------------------------------------------------------------------------
// Safe disconnect
// ---------------------------------------------------------------------------

/// Stops everything if the remote drops out mid-run.
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

void initializeSubsystems([[maybe_unused]] Drivers *drivers) { motorSubsystem.initialize(); }

void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&motorSubsystem);
}

void setDefaultStandardCommands([[maybe_unused]] Drivers *drivers)
{
    // A subsystem's default command runs whenever no other command has claimed it,
    // which for this project means "always".
    motorSubsystem.setDefaultCommand(&motorVelocityCommand);
}

void startStandardCommands(Drivers *drivers)
{
    // Tells the IMU how it is bolted to the board relative to the robot, so that its
    // yaw/pitch/roll come out in the robot's frame rather than the chip's. The 180 here
    // is the board being mounted upside down.
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(0), modm::toRadian(180)));
}

void registerStandardIoMappings([[maybe_unused]] Drivers *drivers)
{
    // Nothing mapped to buttons yet. When you add a second command later, this is where
    // you would bind it -- see tap/control/hold_command_mapping.hpp and friends.
}
}  // namespace standard_control

namespace src::standard
{
void initSubsystemCommands(src::standard::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems(drivers);
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace src::standard

#endif
