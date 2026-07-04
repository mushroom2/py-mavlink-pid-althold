# Gazebo Harmonic + ArduPilot SITL Setup

A step-by-step guide to install **Gazebo Harmonic** and **ArduPilot SITL** (Software-In-The-Loop) on **Ubuntu 24.04 (Noble)** for drone/vehicle simulation experiments.

## Overview

This setup pairs ArduPilot's SITL flight controller with the Gazebo simulator via the
[`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) plugin. The plugin
**does not depend on ROS** — this is a clean SITL-only stack.

| Component        | Version / Choice           |
| ---------------- | -------------------------- |
| OS               | Ubuntu 24.04 (Noble)       |
| Gazebo           | Harmonic (LTS)             |
| Autopilot        | ArduPilot (ArduCopter etc) |
| Plugin           | `ardupilot_gazebo`         |
| GCS              | MAVProxy                   |

> **Why Harmonic?** Harmonic is the LTS Gazebo release with binaries officially provided
> for Ubuntu Noble (24.04), and it is the version ArduPilot recommends for this stack.

## Prerequisites

- Ubuntu 24.04 (Noble), 64-bit
- `git`, `curl`, `sudo` privileges
- A graphics card is recommended (OpenGL / ogre2 render engine)
- Minimum 8 GB RAM recommended

---

## 1. Install ArduPilot + SITL

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Verify plain SITL works **before** adding Gazebo:

```bash
sim_vehicle.py -v ArduCopter -w --map --console
```

Arm and take off to confirm, then close it with `Ctrl+C`. If this works, MAVProxy + SITL
are good.

---

## 2. Install Gazebo Harmonic

```bash
sudo apt update
sudo apt install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic
```

Verify:

```bash
gz sim shapes.sdf
```

You should get a window with some shapes.

> ⚠️ `gz-harmonic` cannot be installed alongside `gazebo11` (gazebo-classic) by default.
> Remove old gazebo-classic first if present.

---

## 3. Build the `ardupilot_gazebo` plugin

Install build dependencies (note **`sim8`** for Harmonic — `sim7` is Garden):

```bash
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

Build:

```bash
export GZ_VERSION=harmonic
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

---

## 4. Configure environment variables

Add the plugin, model, and world paths so Gazebo can locate them:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Run the simulation

Use **two terminals**. Launch Gazebo **first** (with `-r` so physics runs, not paused):

```bash
# Terminal 1
gz sim -v4 -r iris_runway.sdf
```

Then launch SITL bound to Gazebo:

```bash
# Terminal 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

> The frame name must contain `gazebo-` and use `JSON` as the model.

In the MAVProxy console:

```
param set FRAME_CLASS 1   # Quad
param set FRAME_TYPE 1    # X
mode guided
arm throttle
takeoff 5
```

The iris should lift off in the Gazebo window.

---

## Troubleshooting

**`No JSON sensor message received, resending servos`**
SITL is sending servo outputs but getting no physics data back. Causes, in order:
1. Gazebo is **paused** — launch with `-r` or press the play ▶ button.
2. Plugin not loaded — check `echo $GZ_SIM_SYSTEM_PLUGIN_PATH` is set in the terminal
   running `gz sim`; `source ~/.bashrc` and relaunch if empty.
3. Wrong launch order — start Gazebo (playing) first, then SITL.

**`Frame: UNSUPPORTED` / `Arm: Motors: Check frame class and type`**
Frame parameters not set. For the iris quad:
```
param set FRAME_CLASS 1
param set FRAME_TYPE 1
```

**`make -j4` fails**
Almost always the wrong dev package — confirm it is `libgz-sim8-dev` (Harmonic), not `sim7`.

**`gz-sim Unable to find or download file`**
Gazebo can't find the worlds/models. The exported paths didn't take — re-run
`source ~/.bashrc` and relaunch.

**apt dependency conflicts (freetype / libpng `t64`)**
Check that your APT sources point to `noble`, not `jammy`. Inspect
`/etc/apt/sources.list.d/ubuntu.sources` and any files under `sources.list.d/`.

---

## References

- [ArduPilot: Using SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo Harmonic install (Ubuntu)](https://gazebosim.org/docs/harmonic/install_ubuntu/)