# Unitree G1 Dual Dex1-1 Arm and Hand Motion

Low-level Unitree SDK2 example for a Unitree G1 with left and right Dex1-1 grippers.

The program:

- subscribes to the G1 low-level state topic `rt/lowstate`
- publishes G1 low-level joint commands to `rt/lowcmd`
- publishes Dex1-1 gripper commands to `rt/dex1/left/cmd` and `rt/dex1/right/cmd`
- reads Dex1-1 state from `rt/dex1/left/state` and `rt/dex1/right/state`
- moves both arms from the current pose to a raised pose
- opens and closes both grippers while holding the arms
- returns both arms to the lowered pose

## Repository contents

```text
.
├── CMakeLists.txt
├── README.md
└── src/
    └── g1_dual_dex1_arm_hand_motion.cpp
```

The original source file was renamed from `dual_dex1_arm_hand_motion_g1.cpp` to `g1_dual_dex1_arm_hand_motion.cpp` so the filename matches the robot, hand, and behavior more clearly.

## Requirements

- Unitree G1 robot
- Left and right Unitree Dex1-1 grippers
- Linux development machine connected to the robot network
- Unitree SDK2 cloned locally
- CMake and a C++17 compiler

## Build

Clone Unitree SDK2 and this repository:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/notalexander30/unitree-g1_dex1-1-dual_motion.git
cd unitree-g1_dex1-1-dual_motion
```

Configure the build by pointing `UNITREE_SDK2_ROOT` at your local SDK checkout:

```bash
cmake -S . -B build -DUNITREE_SDK2_ROOT=/absolute/path/to/unitree_sdk2
cmake --build build -j
```

You can also use an environment variable:

```bash
export UNITREE_SDK2_ROOT=/absolute/path/to/unitree_sdk2
cmake -S . -B build
cmake --build build -j
```

## Run

Find the network interface connected to the robot:

```bash
ip link
```

Run the example with that interface name:

```bash
./build/g1_dual_dex1_arm_hand_motion eth0
```

Replace `eth0` with your robot-facing interface, for example `enp3s0`, `eno1`, or another interface shown by `ip link`.

## Safety checklist

- Make sure the robot is supported and has enough free space around both arms.
- Keep an emergency stop available.
- Confirm that no high-level sport/motion service is actively controlling the robot before running low-level commands.
- Start with conservative gripper targets and arm poses if your hardware calibration differs.
- Stop immediately if the arms or grippers move in an unexpected direction.

## Important tuning values

The Dex1-1 gripper target values are near the top of the source file:

```cpp
constexpr float kDex1OpenQ = 0.30f;
constexpr float kDex1ClosedQ = 4.00f;
constexpr float kDex1Kp = 5.0f;
constexpr float kDex1Kd = 0.05f;
```

Tune `kDex1OpenQ` and `kDex1ClosedQ` for your gripper calibration before running this on different hardware.

The arm motion is generated in `MakeRaisedArmPose()` by offsetting the current shoulder and elbow positions. Adjust those offsets if your robot needs a smaller or different movement.

## SDK example integration

If you prefer to build directly inside `unitree_sdk2`, copy the source file into the SDK examples folder:

```bash
cp src/g1_dual_dex1_arm_hand_motion.cpp /absolute/path/to/unitree_sdk2/example/g1/low_level/
```

Then add this to `/absolute/path/to/unitree_sdk2/example/g1/CMakeLists.txt`:

```cmake
add_executable(g1_dual_dex1_arm_hand_motion low_level/g1_dual_dex1_arm_hand_motion.cpp)
target_link_libraries(g1_dual_dex1_arm_hand_motion unitree_sdk2)
```

Build from the SDK root:

```bash
cd /absolute/path/to/unitree_sdk2
cmake -S . -B build
cmake --build build -j
./build/bin/g1_dual_dex1_arm_hand_motion eth0
```
