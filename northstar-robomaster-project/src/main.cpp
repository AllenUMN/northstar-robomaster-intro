/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarFleet2025.
 *
 * NorthStarFleet2025 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarFleet2025 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"

/* robot includes ---------------------------------------------------------*/
#include "tap/communication/gpio/pwm.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(tap::Drivers::DT);

using namespace src::standard;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers *drivers);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            // Fuses the latest accel/gyro samples into an orientation. Must run before
            // the scheduler so that any control code sees fresh IMU data this tick.
            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());

            // Runs every subsystem's refresh() and every scheduled command's execute().
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            // Packs whatever the subsystems asked for into CAN frames and sends them.
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
        }

        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(Drivers *drivers)
{
    // things we need to check controller
    drivers->remote.initialize();
    drivers->analog.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->pwm.init();

    // if controller is on when the robot turns on, wait for it to be off.
    // This is to prevent the shredding of wires
    modm::delay_ms(3000);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    int i = 0;
    while (i < 5000)
    {
        drivers->remote.read();
        if (drivers->remote.isConnected())
        {
            i = 0;
            drivers->pwm.write(0.5f, tap::gpio::Pwm::Buzzer);
            drivers->pwm.setTimerFrequency(tap::gpio::Pwm::TIMER4, 1500);
        }
        else
        {
            i++;
            drivers->pwm.write(0.0f, tap::gpio::Pwm::Buzzer);
        }

        modm::delay_us(10);
    }

    drivers->leds.set(tap::gpio::Leds::Blue, true);

    drivers->can.initialize();
    drivers->errorController.init();

    // The BMI088 is the board's onboard IMU. Nothing in the intro exercise uses it yet,
    // but it is initialized here so that follow-on exercises (holding a position, or
    // driving the motor from the gyro the way the turret holds its heading while the
    // chassis moves underneath it) have working orientation data to build on.
    drivers->bmi088.initialize(500, 0.05f, 0.000f);
    drivers->bmi088.setTargetTemperature(35.0f);
    drivers->bmi088.setCalibrationSamples(2000);
}

/*
 * Handy to watch in the debugger while working with the IMU. Degrees, and rad/s.
 */
float debugYaw = 0.0f;
float debugPitch = 0.0f;
float debugRoll = 0.0f;
float debugYawRate = 0.0f;

static bool imuCalibrationRequested = false;

static void updateIo(Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    // Pulls motor feedback (position, velocity, torque) off the CAN bus.
    drivers->canRxHandler.pollCanData();

    // Pulls raw accel/gyro samples off SPI.
    drivers->bmi088.read();

    drivers->remote.read();

    // Zero the gyro once, on the first tick where the remote is alive. The robot must be
    // sitting still for this -- calibration averages samples to find the gyro's bias, so
    // any motion during it gets baked in as permanent drift.
    if (!imuCalibrationRequested && drivers->remote.isConnected())
    {
        drivers->bmi088.requestCalibration();
        imuCalibrationRequested = true;
    }

    debugYaw = modm::toDegree(drivers->bmi088.getYaw());
    debugPitch = modm::toDegree(drivers->bmi088.getPitch());
    debugRoll = modm::toDegree(drivers->bmi088.getRoll());
    debugYawRate = drivers->bmi088.getGz();
}
