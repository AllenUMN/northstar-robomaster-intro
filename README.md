# NorthStar Intro Project — Spin a GM6020

Welcome. This is your first project on the robot codebase.

A **GM6020 is a motor** — a DJI brushless gimbal motor with a built-in driver and absolute
encoder, commanded over CAN rather than by a raw PWM signal. It is the motor that drives the
turret yaw axis on the competition robots.

By the end you will have made one spin under joystick control, using the same command-based
structure the competition robots use. The repo has been stripped down to almost nothing so
that the only code you have to understand is the code you are writing.

---

## Setup

Follow [the Northstar Docker setup guide](https://github.com/Northstar-Advanced-Robotics/resources/blob/david/refactor/setup/docker_setup.md)
to get your machine into the dev container.

That guide is the whole install. Once VS Code reopens in the container you are done: the ARM
toolchain, Python, pipenv, clang-format and GoogleTest are already inside the image, and the
project's Python dependencies install themselves the first time the container comes up.
Nothing below this line needs any setup beyond that.

---

## The idea: subsystems and commands

The robot code is built out of two kinds of thing.

**A Subsystem is a piece of hardware.** It owns the motor and is the only code allowed to
talk to it. It knows *how* to do things — how to run a PID loop, how to convert units, how
to keep itself safe when something is unplugged.

**A Command is a behavior.** It decides *what* the robot should be doing right now. It
reads operator input and tells subsystems what it wants. It never touches motors directly.

A scheduler ties them together. Every tick (every `tap::Drivers::DT` milliseconds) it:

1. calls `execute()` on every scheduled command, then
2. calls `refresh()` on every registered subsystem.

The scheduler also guarantees that **only one command may control a given subsystem at a
time**. That is the whole point of the structure: without it, two behaviors that both want
the chassis would fight, writing conflicting outputs on alternating ticks.

In this project the control loop is *driven by the command*: `MotorVelocityCommand::execute()`
calls `MotorSubsystem::runVelocityPid()` every tick, and the subsystem's `refresh()` is
empty. That is how the turret works in the real codebase — the turret subsystem holds the
motors, while whichever command is currently aiming runs the controller.

It matters because of where this goes next. Once velocity control works, a *position*
controller, or one that holds a heading using the onboard IMU (the way the turret stays
pointed while the chassis spins underneath it), is just another method on the subsystem
plus another command. You pick between them by scheduling a different command; the
subsystem never needs to know which mode it is in.

---

## Your task

Make a GM6020 spin at a speed set by the left stick, with a velocity PID holding that speed.

The files are already created and wired together. The build compiles right now and will
flash to a board — the motor just won't move. Your job is to fill in four `TODO(student)`
blocks.

Every path in this section is relative to `northstar-robomaster-project/`. (The `taproot/`
directory at the repo root is empty; the vendored copy of taproot lives inside the project
folder.)

### 1. `src/control/motor/motor_subsystem.cpp` → `getCurrentRpm()`

Report how fast the motor is actually spinning, in RPM.

`motor.getEncoder()->getVelocity()` gives you the speed, but **not in RPM**. Go read the
doc comment on `getVelocity()` in
`taproot/src/tap/communication/sensors/encoder/encoder_interface.hpp` and convert.

Get this wrong and the PID still "works" — it just regulates a number that's off by a
constant factor from what you think, and your gains come out looking absurd. This is the
most common way this exercise goes sideways.

### 2. `src/control/motor/motor_subsystem.cpp` → `runVelocityPid()`

Close the loop:

1. If `motor.isMotorOnline()` is false, call `stop()` and return early.
2. Compute the error: target minus current.
3. `velocityPid.runControllerDerivateError(error, dt)`, passing `tap::Drivers::DT` as `dt`.
4. Write the result with `motor.setDesiredOutput(...)`.

Step 1 is not optional bookkeeping. Skip it and the integral term winds up while the motor
is unplugged, so the motor lurches at full output the moment it reconnects.

### 3. `src/control/motor/motor_velocity_command.cpp` → `execute()`

One line. `operatorInterface->getMotorVelocityInput()` returns a number in `[-1, 1]`;
`motor->runVelocityPid()` wants RPM; `MAX_MOTOR_RPM` is what full stick should mean.

This call is what actually steps the control loop. The subsystem does nothing on its own,
so if `execute()` is empty the motor never moves no matter how good your PID is.

### 4. `src/robot/standard/standard_motor_constants.hpp` → PID gains

All three gains are `0.0f`, so the motor will not move even once the code above is right.
Tune on hardware:

1. Raise `kp` until the motor roughly reaches the speed you asked for.
2. Add `kd` if it oscillates or overshoots.
3. Add `ki` last, and only if it consistently settles short of the target.

Leave `maxOutput` alone — it is already correct, and the comment there explains why.

---

## Programming, testing and deploying

Everything here runs in the container. There is nothing to install first.

The two things you do most often are VS Code tasks, so you can just hit a button:

- **Build** — <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>B</kbd>, or
  <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> → *Tasks: Run Task* → **Build Standard**
- **Test** — <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> → *Tasks: Run Task* → **Run Tests**

The equivalent commands, from `northstar-robomaster-project/`:

```
pipenv run scons build profile=debug robot=STANDARD      # build the firmware
pipenv run scons run-tests profile=fast robot=STANDARD   # build and run the unit tests
```

`STANDARD` is the only robot target in this repo. Asking for any other one is an error, and
`robot=TARGET_STANDARD` does not work either — the argument is matched as a substring of the
robot name, so spelling out the `TARGET_` prefix silently drops you into an interactive
prompt.

### The tests

`test/motor_subsystem_tests.cpp` checks the subsystem half of the exercise: safe-disconnect
behavior, the offline guard, and that the PID pushes the output in the right direction.

**These tests fail when you start.** That is intentional — a green suite is the definition
of "done" for steps 1 and 2. They build their own PID gains rather than reading your
tuned ones, so they test your control loop, not your tuning.

CI only checks that the tests *compile*, so you will not be blocked by a red build while
you work.

### Deploying

The container builds the firmware; it does not flash it. Flashing and on-target debugging
happen on your host machine, in SEGGER Ozone, against the `.elf` the container just produced
at:

```
northstar-robomaster-project/build/hardware/scons-debug/TARGET_STANDARD/northstar-robomaster-project.elf
```

See [docs/ozone-debugging.md](docs/ozone-debugging.md) for how to point Ozone at it and get
breakpoints binding.

---

## Checking your work on hardware

You need a control board (the MCB) and a GM6020 motor on **CAN1**, with the dial on the
back of the motor set to position **1**. (Dial position sets the CAN ID: position 1 is
`0x205`, which taproot calls `MOTOR5`. If the motor doesn't respond, check this first — it's
in `standard_motor_constants.hpp` if you need a different one.)

It's working when:

- Pushing the left stick forward spins the motor, and further forward spins it faster.
- The motor **holds its speed when you load it by hand** — that's the PID doing its job. An
  open-loop output would just slow down.
- Releasing the stick coasts it to a stop.
- Turning off the remote stops the motor immediately (`RemoteSafeDisconnectFunction`).

**Warning:** a GM6020 has real torque. Clamp it down and keep fingers and cables clear
before you power it.

---

## Where things live

Rooted at `northstar-robomaster-project/`:

```
src/
  main.cpp                          board init and the main loop
  drivers_singleton.{hpp,cpp}       the one global Drivers instance
  control/
    motor/
      motor_subsystem.{hpp,cpp}     >>> the motor + its PID (steps 1, 2)
      motor_velocity_command.*      >>> joystick -> target speed (step 3)
    safe_disconnect.{hpp,cpp}       stops everything if the remote drops
  robot/
    control_operator_interface.*    all remote reads live here, nowhere else
    standard/
      standard_control.cpp          where the robot gets assembled
      standard_motor_constants.hpp  >>> IDs, limits, PID gains (step 4)
      standard_drivers.hpp
test/
  motor_subsystem_tests.cpp
```

Read `standard_control.cpp` early. It is short, and it shows the whole pattern: declare
subsystems, declare commands, register them, set a default command. Every real robot in the
competition codebase is that same file, just much longer.

## A note on `ControlOperatorInterface`

Commands never read `drivers->remote` directly. They ask `ControlOperatorInterface`
instead, so that "which stick does what" is defined in exactly one file. When you want to
change the control scheme, you change it there and every command follows.

It is already written for you — `getMotorVelocityInput()` reads the left stick and applies
a deadzone. Read it; it's about ten lines, and it's the model for how you'd add a second
axis later.

## The IMU

The board's BMI088 is initialized and running (`main.cpp`), calibrated once at startup, and
oriented to the robot's frame via `setMountingTransform()` in `standard_control.cpp`.
Nothing in this exercise uses it. It is live so that the follow-on exercises — position
control, or holding a heading against chassis motion — have working orientation data to
build on. `drivers->bmi088.getYaw()`, `getPitch()`, `getRoll()`, and `getGz()` are what you
would reach for; the `debugYaw`/`debugPitch`/`debugRoll`/`debugYawRate` globals in
`main.cpp` are there to watch in the debugger.

Calibration averages gyro samples to find their bias, so **the robot must be sitting still**
during the first second or so after the remote connects, or the bias is wrong and yaw
drifts.
