# Unitree G1 With Dex1-1 Dual Motion

Low-level Unitree SDK2 example for a Unitree G1 robot with left and right Dex1-1 grippers.

The program:

- initializes Unitree DDS on the network interface you pass on the command line
- subscribes to the G1 low-level state topic `rt/lowstate`
- publishes G1 low-level joint commands to `rt/lowcmd`
- publishes Dex1-1 gripper commands to `rt/dex1/left/cmd` and `rt/dex1/right/cmd`
- reads Dex1-1 gripper state from `rt/dex1/left/state` and `rt/dex1/right/state`
- moves both G1 arms from the current pose to a raised pose
- opens and closes both Dex1-1 grippers while holding the arms
- returns both arms to the lowered pose

## Safety First

- Use this only with a supported Unitree G1 plus Dex1-1 setup.
- Keep an emergency stop available and reachable.
- Make sure the robot has free space around both arms and hands.
- Keep people, cables, tools, and loose objects away from the arms and grippers.
- Run only one controller at a time. Stop high-level sport/motion services, other SDK examples, ROS nodes, and custom control programs before running this demo.
- Do not run if the program cannot receive `rt/lowstate`.
- Check and tune the gripper open/closed values before using different hardware or calibration.
- Stop immediately if either arm or gripper moves in an unexpected direction.

## Repository Contents

```text
.
|-- CMakeLists.txt
|-- README.md
`-- src/
    `-- g1_dual_dex1_arm_hand_motion.cpp
```

## Requirements

- Linux laptop or development PC.
- Unitree G1 connected to the same robot network.
- Left and right Unitree Dex1-1 grippers.
- Unitree SDK2 cloned locally.
- Git.
- CMake 3.16 or newer.
- C++17 compiler, usually `g++`.
- Basic Linux network tools: `iproute2`.

Install common build tools:

```bash
sudo apt update
sudo apt install -y git cmake build-essential iproute2
```

## First Setup On A Linux Laptop

### 1. Clone Unitree SDK2

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
```

If you already have SDK2, use that existing path instead.

### 2. Clone this project

```bash
cd ~
git clone https://github.com/notalexander30/unitree-g1_dex1-1-dual_motion.git
cd unitree-g1_dex1-1-dual_motion
```

Always run the build commands from this project directory.

### 3. Point the project at SDK2

Use the absolute path to your SDK2 checkout:

```bash
export UNITREE_SDK2_ROOT=$HOME/unitree_sdk2
```

If your SDK is somewhere else:

```bash
export UNITREE_SDK2_ROOT=/absolute/path/to/unitree_sdk2
```

### 4. Build

```bash
cmake -S . -B build -DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT"
cmake --build build -j
```

The executable should be created at:

```text
build/g1_dual_dex1_arm_hand_motion
```

### 5. Connect the robot network

1. Connect the laptop to the robot-facing Ethernet network.
2. Power on the robot and Dex1-1 grippers.
3. Confirm the emergency stop is ready.
4. Stop any other controller that could command the robot.

### 6. Find the robot-facing network interface

```bash
ip link
ip addr
```

Look for the wired interface connected to the robot, for example `eth0`, `enp3s0`, `enx...`, or `eno1`.

If the interface is stuck, reset it:

```bash
sudo ip link set <interface> down
sudo ip link set <interface> up
ip addr show <interface>
```

Replace `<interface>` with your actual interface name.

If your Unitree network setup requires a static IP address, set the IP according to your Unitree network documentation before running this program.

### 7. Run a first state check

Run the executable with your robot-facing interface:

```bash
./build/g1_dual_dex1_arm_hand_motion <interface>
```

Example:

```bash
./build/g1_dual_dex1_arm_hand_motion eth0
```

The program should print:

```text
Initializing DDS on network interface: <interface>
Waiting for G1 lowstate...
Received G1 lowstate.
```

If it times out waiting for G1 lowstate, do not retry motion yet. Go to [Troubleshooting](#troubleshooting).

## Daily Start

```bash
cd ~/unitree-g1_dex1-1-dual_motion
export UNITREE_SDK2_ROOT=$HOME/unitree_sdk2
cmake --build build -j
ip link
sudo ip link set <interface> up
./build/g1_dual_dex1_arm_hand_motion <interface>
```

## Important Tuning Values

Dex1-1 gripper targets are near the top of `src/g1_dual_dex1_arm_hand_motion.cpp`:

```cpp
constexpr float kDex1OpenQ = 0.30f;
constexpr float kDex1ClosedQ = 4.00f;
constexpr float kDex1Kp = 5.0f;
constexpr float kDex1Kd = 0.05f;
```

Tune `kDex1OpenQ` and `kDex1ClosedQ` for your gripper calibration.

The raised arm pose is generated in `MakeRaisedArmPose()` by offsetting the current shoulder and elbow joint values:

```cpp
pose.at(LeftShoulderPitch) = start_pose.at(LeftShoulderPitch) - 0.80f;
pose.at(LeftShoulderRoll) = start_pose.at(LeftShoulderRoll) + 0.25f;
pose.at(LeftElbow) = start_pose.at(LeftElbow) - 1.20f;
```

Reduce these offsets before first testing on an unfamiliar robot.

## SDK Example Integration

If you prefer to build inside `unitree_sdk2`, copy the source file into the SDK examples folder:

```bash
cp src/g1_dual_dex1_arm_hand_motion.cpp "$UNITREE_SDK2_ROOT/example/g1/low_level/"
```

Then add this to `$UNITREE_SDK2_ROOT/example/g1/CMakeLists.txt`:

```cmake
add_executable(g1_dual_dex1_arm_hand_motion low_level/g1_dual_dex1_arm_hand_motion.cpp)
target_link_libraries(g1_dual_dex1_arm_hand_motion unitree_sdk2)
```

Build and run from the SDK root:

```bash
cd "$UNITREE_SDK2_ROOT"
cmake -S . -B build
cmake --build build -j
./build/bin/g1_dual_dex1_arm_hand_motion <interface>
```

## Troubleshooting

| Problem or return message | What to do first |
| --- | --- |
| `git: command not found` | Run `sudo apt update && sudo apt install -y git`. |
| `cmake: command not found` | Run `sudo apt update && sudo apt install -y cmake`. |
| `g++: command not found` or compiler errors about no C++ compiler | Run `sudo apt install -y build-essential`. |
| `Set UNITREE_SDK2_ROOT to your local unitree_sdk2 checkout.` | Export the SDK path: `export UNITREE_SDK2_ROOT=$HOME/unitree_sdk2`, then rerun CMake. |
| CMake cannot find SDK files | Check `ls "$UNITREE_SDK2_ROOT"` and make sure it points to the SDK2 root, not a subfolder. |
| Build fails after moving SDK2 | Delete and recreate the build folder: `rm -rf build && cmake -S . -B build -DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT"`. |
| Program prints `Usage: g1_dual_dex1_arm_hand_motion network_interface` | Run it with an interface name, for example `./build/g1_dual_dex1_arm_hand_motion eth0`. |
| You do not know the interface name | Run `ip link` and `ip addr`. Look for the Ethernet interface connected to the robot. |
| Interface is `DOWN` | Run `sudo ip link set <interface> up`, then retry. |
| Interface does not appear | Check Ethernet cable, adapter, USB Ethernet dongle, and power. Replug the adapter and run `ip link` again. |
| `Waiting for G1 lowstate...` then timeout | Check the interface name, robot power, robot network IP setup, and that the laptop is connected to the robot-facing network. Do not run motion until lowstate is received. |
| `Failed to switch to Release Mode.` | A high-level motion mode may still be active. Stop other Unitree services/examples and retry only when the robot is safe. |
| `[ERROR] G1 lowstate CRC error` | Stop the program. Check SDK version compatibility, robot state topic, and network stability. |
| Arms do not move but lowstate is received | Confirm the robot is in a mode that accepts low-level commands and no other controller is taking over `rt/lowcmd`. |
| Grippers do not move | Confirm Dex1-1 hardware is connected and using `rt/dex1/left/cmd` and `rt/dex1/right/cmd`. Check the open/closed q targets. |
| Grippers move backward | Swap or retune `kDex1OpenQ` and `kDex1ClosedQ` for your calibration. |
| Motion is too large | Reduce the shoulder and elbow offsets in `MakeRaisedArmPose()` before running again. |
| Motion is jerky | Check network stability and reduce gains or movement offsets before more testing. |
| Another program is controlling the robot | Stop all other Unitree SDK examples, ROS nodes, dashboards, and motion services before retrying. |
| You are on Wi-Fi | Use wired Ethernet for the robot network. DDS low-level control should not depend on Wi-Fi. |
| You are unsure whether it is safe | Do not run the executable. Verify hardware, network, robot mode, and emergency stop first. |

## When You Are Done

Stop the program with `Ctrl+C` if it is still running.

Optionally bring the robot-facing interface down:

```bash
sudo ip link set <interface> down
```

Power off or secure the robot according to your lab procedure.
