# Debugging on hardware with Ozone

The dev container builds the firmware; it does not flash it. Flashing and on-target debugging
happen on your Windows host, in SEGGER Ozone, against the `.elf` the container produced.

This guide gets Ozone to open that `.elf` with working breakpoints and source navigation. Most
of it is about one thing: telling Ozone where the sources live, since the paths baked into the
debug info are container paths that mean nothing to Windows.

## Prerequisites

- A working dev container — see
  [the Northstar Docker setup guide](https://github.com/Northstar-Advanced-Robotics/resources/blob/david/refactor/setup/docker.md).
- [SEGGER Ozone](https://www.segger.com/products/debug-probes/j-link/tools/ozone/) and the
  J-Link software/drivers installed on Windows.
- A J-Link probe connected to the target board.

## 1. Know how your repo is mounted

Ozone needs a Windows-visible path to the `.elf` and to the sources. Which path that is depends
on how you opened the repo in a container.

### If you cloned normally and used "Reopen in Container"

This is the flow the setup guide describes, and it is the simple case. The repo is bind-mounted
from your Windows filesystem, so the build output is already sitting at an ordinary Windows path
inside the folder you cloned into, for example:

```
C:\Users\you\dev\northstar-robomaster-intro\northstar-robomaster-project\build\...
```

Nothing else to look up. Skip to step 2.

### If you used "Clone Repository in Named Container Volume..."

The repo lives inside a Docker named volume rather than on your filesystem. Docker Desktop's
WSL2 backend keeps volumes on a second virtual disk, reachable from Windows over the network
provider as:

```
\wsl.localhost\docker-desktop\mnt\docker-desktop-disk\data\docker\volumes\<volume-name>\_data\northstar-robomaster-intro
```

Confirm it resolves, from a normal PowerShell window (the `docker-desktop` distro must be
running, which it is whenever Docker Desktop is):

```powershell
Test-Path "\wsl.localhost\docker-desktop\mnt\docker-desktop-disk\data\docker\volumes\<volume-name>\_data"
```

**Use a short volume name** (e.g. `rmb`) when you create it. Ozone resolves each source file by
joining a substituted prefix with a relative path, and this prefix is already long. The plain
**Clone Repository in Container Volume...** command auto-names the volume something like
`vsc-northstar-robomaster-<64-char-hash>`, which pushes the combined path past Windows'
260-character `MAX_PATH`. When that happens Ozone fails *silently*: `File.Open` on the `.elf`
works fine, but the Functions window stays empty and no breakpoint ever binds. A short name
avoids the problem outright — no NTFS symlinks or mapped drive letters needed.

## 2. Build the firmware

In the container terminal:

```
cd northstar-robomaster-project
pipenv run scons build profile=debug robot=STANDARD
```

`STANDARD` is the only robot target in this repo. Note that `robot=TARGET_STANDARD` does not
work — the argument is matched as a substring of the robot name, so it silently drops to an
interactive prompt.

The `.elf` lands at:

```
northstar-robomaster-project/build/hardware/scons-debug/TARGET_STANDARD/northstar-robomaster-project.elf
```

Two parts of that path move with your build arguments. `profile=` sets the `scons-debug`
component, and adding `profiling=true` changes `hardware` to `hardware-profiling`.

## 3. Configure the Ozone project

Create or edit your `.jdebug` project file with a path substitution and the `.elf` to open.

The substitution's *old* path has to match the `comp_dir` baked into the DWARF debug info at
compile time. Note that this is the **project** directory, not the repo root — scons runs from
`northstar-robomaster-project/`, so it is:

```
/workspaces/northstar-robomaster-intro/northstar-robomaster-project
```

Rather than trusting this document, read it off your own `.elf` from inside the container:

```
arm-none-eabi-readelf --debug-dump=info <elf-file> | grep DW_AT_comp_dir | head -1
```

Getting this wrong by one directory level is the usual cause of an empty Functions window.

For a named-volume clone:

```
Project.AddPathSubstitute ("/workspaces/northstar-robomaster-intro/northstar-robomaster-project", "//wsl.localhost/docker-desktop/mnt/docker-desktop-disk/data/docker/volumes/<volume-name>/_data/northstar-robomaster-intro/northstar-robomaster-project");

File.Open ("//wsl.localhost/docker-desktop/mnt/docker-desktop-disk/data/docker/volumes/<volume-name>/_data/northstar-robomaster-intro/northstar-robomaster-project/build/hardware/scons-debug/TARGET_STANDARD/northstar-robomaster-project.elf");
```

For a bind-mounted clone, the second argument is just your Windows clone folder, with forward
slashes:

```
Project.AddPathSubstitute ("/workspaces/northstar-robomaster-intro/northstar-robomaster-project", "C:/Users/you/dev/northstar-robomaster-intro/northstar-robomaster-project");

File.Open ("C:/Users/you/dev/northstar-robomaster-intro/northstar-robomaster-project/build/hardware/scons-debug/TARGET_STANDARD/northstar-robomaster-project.elf");
```

Reload the project in Ozone. The Functions window should populate, and breakpoints set in
source should bind and hit.

## Troubleshooting

- **`Error (7): File could not be opened for reading`** on `File.Open` — the `docker-desktop`
  distro was likely still starting up (e.g. right after waking the PC or starting Docker
  Desktop). Check with `wsl --list --running` and `Test-Path` on the target file before
  retrying in Ozone; it typically resolves within a few seconds once Docker Desktop is fully
  up.
- **Functions window still empty** — double check the `AddPathSubstitute` old and new paths
  match exactly (no trailing slash mismatches, correct volume name), and that you fully closed
  and reopened the `.jdebug` project rather than just saving it. If your volume name is longer
  than a handful of characters, you may be back over `MAX_PATH` — shortening it is the fix.
- **`arm-none-eabi-gdb: error while loading shared libraries: libncurses.so.5`** when trying to
  inspect an `.elf` from inside the container — the prebuilt ARM toolchain's `gdb` needs the
  older `libncurses5`, which is not installed by default:
  ```
  sudo apt-get update && sudo apt-get install -y libncurses5
  ```
  Or skip `gdb` and use `arm-none-eabi-readelf --debug-dump=info` instead, which has no such
  dependency.
