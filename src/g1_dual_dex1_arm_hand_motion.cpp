#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace std;
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

using Dex1CmdMsg = unitree_go::msg::dds_::MotorCmds_;
using Dex1StateMsg = unitree_go::msg::dds_::MotorStates_;

static const string HG_CMD_TOPIC = "rt/lowcmd";
static const string HG_STATE_TOPIC = "rt/lowstate";
static const string LEFT_DEX1_CMD_TOPIC = "rt/dex1/left/cmd";
static const string RIGHT_DEX1_CMD_TOPIC = "rt/dex1/right/cmd";
static const string LEFT_DEX1_STATE_TOPIC = "rt/dex1/left/state";
static const string RIGHT_DEX1_STATE_TOPIC = "rt/dex1/right/state";

constexpr int G1_NUM_MOTOR = 29;
constexpr float CONTROL_DT_SECONDS = 0.02f;

// Dex1-1 gripper targets for both left and right grippers.
constexpr float kDex1OpenQ = 0.30f;
constexpr float kDex1ClosedQ = 4.00f;
constexpr float kDex1Kp = 5.0f;
constexpr float kDex1Kd = 0.05f;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    unique_lock<shared_mutex> lock(mutex_);
    data_ = make_shared<T>(new_data);
  }

  shared_ptr<const T> GetData() const {
    shared_lock<shared_mutex> lock(mutex_);
    return data_;
  }

 private:
  shared_ptr<T> data_;
  mutable shared_mutex mutex_;
};

struct MotorState {
  array<float, G1_NUM_MOTOR> q = {};
  array<float, G1_NUM_MOTOR> dq = {};
};

enum class Mode : uint8_t {
  PR = 0,
  AB = 1
};

enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleRoll = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleRoll = 11,
  WaistYaw = 12,
  WaistRoll = 13,
  WaistPitch = 14,
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,
  LeftWristYaw = 21,
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,
  RightWristYaw = 28
};

const array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40};

const array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1};

inline uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  const uint32_t dw_polynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= dw_polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) {
        crc32 ^= dw_polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc32;
}

float Lerp(float start, float target, float ratio) {
  return start * (1.0f - ratio) + target * ratio;
}

float SmoothStep(float ratio) {
  ratio = clamp(ratio, 0.0f, 1.0f);
  return ratio * ratio * (3.0f - 2.0f * ratio);
}

class G1DualArmDex1GripperExample {
 public:
  explicit G1DualArmDex1GripperExample(const string& network_interface)
      : mode_pr_(Mode::PR), mode_machine_(0) {
    cout << "Initializing DDS on network interface: " << network_interface << endl;
    ChannelFactory::Instance()->Init(0, network_interface);
    ReleaseMotionMode();
    InitLowLevel();
    InitDex1Grippers();
  }

  bool WaitForG1State(double timeout_seconds = 5.0) {
    cout << "\nWaiting for G1 lowstate..." << endl;
    const auto start_time = chrono::steady_clock::now();

    while (true) {
      if (motor_state_buffer_.GetData()) {
        cout << "Received G1 lowstate." << endl;
        return true;
      }

      const auto now = chrono::steady_clock::now();
      const double elapsed = chrono::duration<double>(now - start_time).count();
      if (elapsed > timeout_seconds) {
        cout << "ERROR: Timed out waiting for G1 lowstate." << endl;
        return false;
      }

      this_thread::sleep_for(chrono::milliseconds(20));
    }
  }

  void RunDemoSequence() {
    const auto current_state_ptr = motor_state_buffer_.GetData();
    if (!current_state_ptr) {
      cout << "No G1 state available. Skipping motion." << endl;
      return;
    }

    const MotorState start_state = *current_state_ptr;
    array<float, G1_NUM_MOTOR> lowered_pose = start_state.q;
    array<float, G1_NUM_MOTOR> raised_pose = MakeRaisedArmPose(lowered_pose);

    cout << "\nSequence:" << endl;
    cout << "1) lift both G1 arms to a curl-like hold" << endl;
    cout << "2) open, close, open, close both Dex1-1 grippers while holding arms" << endl;
    cout << "3) drop both arms and finish" << endl;

    MoveArmsBetweenPoses(lowered_pose, raised_pose, 2.5f, false, 0.0f, 0.0f);
    HoldArmPose(raised_pose, 0.5f, false, 0.0f, 0.0f);

    HoldArmPose(raised_pose, 1.0f, true, kDex1OpenQ, kDex1OpenQ);
    HoldArmPose(raised_pose, 1.0f, true, kDex1ClosedQ, kDex1ClosedQ);
    HoldArmPose(raised_pose, 1.0f, true, kDex1OpenQ, kDex1OpenQ);
    HoldArmPose(raised_pose, 1.0f, true, kDex1ClosedQ, kDex1ClosedQ);

    MoveArmsBetweenPoses(raised_pose, lowered_pose, 3.0f, true, kDex1ClosedQ, kDex1ClosedQ);

    cout << "\nDone. Arms are back down and both Dex1-1 grippers finished closed." << endl;
  }

 private:
  Mode mode_pr_;
  uint8_t mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;
  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelPublisherPtr<Dex1CmdMsg> left_dex1_cmd_publisher_;
  ChannelPublisherPtr<Dex1CmdMsg> right_dex1_cmd_publisher_;
  ChannelSubscriberPtr<Dex1StateMsg> left_dex1_state_subscriber_;
  ChannelSubscriberPtr<Dex1StateMsg> right_dex1_state_subscriber_;
  shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_client_;

  void ReleaseMotionMode() {
    cout << "\nChecking active motion-control mode..." << endl;
    motion_switcher_client_ = make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_client_->SetTimeout(5.0f);
    motion_switcher_client_->Init();

    string form;
    string name;
    while (motion_switcher_client_->CheckMode(form, name), !name.empty()) {
      cout << "Active motion mode detected: " << name << endl;
      if (motion_switcher_client_->ReleaseMode()) {
        cout << "Failed to switch to Release Mode." << endl;
      }
      sleep(2);
    }
    cout << "No active motion mode detected, or already released." << endl;
  }

  void InitLowLevel() {
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        bind(&G1DualArmDex1GripperExample::LowStateHandler, this, placeholders::_1), 1);
  }

  void InitDex1Grippers() {
    left_dex1_cmd_publisher_.reset(new ChannelPublisher<Dex1CmdMsg>(LEFT_DEX1_CMD_TOPIC));
    right_dex1_cmd_publisher_.reset(new ChannelPublisher<Dex1CmdMsg>(RIGHT_DEX1_CMD_TOPIC));
    left_dex1_cmd_publisher_->InitChannel();
    right_dex1_cmd_publisher_->InitChannel();

    left_dex1_state_subscriber_.reset(new ChannelSubscriber<Dex1StateMsg>(LEFT_DEX1_STATE_TOPIC));
    right_dex1_state_subscriber_.reset(new ChannelSubscriber<Dex1StateMsg>(RIGHT_DEX1_STATE_TOPIC));
    left_dex1_state_subscriber_->InitChannel(
        bind(&G1DualArmDex1GripperExample::LeftDex1StateHandler, this, placeholders::_1), 1);
    right_dex1_state_subscriber_->InitChannel(
        bind(&G1DualArmDex1GripperExample::RightDex1StateHandler, this, placeholders::_1), 1);

    cout << "Dex1-1 command topics ready:" << endl;
    cout << "  " << LEFT_DEX1_CMD_TOPIC << endl;
    cout << "  " << RIGHT_DEX1_CMD_TOPIC << endl;
  }

  void LowStateHandler(const void* message) {
    LowState_ low_state = *(const LowState_*)message;
    if (low_state.crc() != Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      cout << "[ERROR] G1 lowstate CRC error" << endl;
      return;
    }

    MotorState state;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      state.q.at(i) = low_state.motor_state()[i].q();
      state.dq.at(i) = low_state.motor_state()[i].dq();
    }
    motor_state_buffer_.SetData(state);

    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) {
        cout << "G1 type: " << unsigned(low_state.mode_machine()) << endl;
      }
      mode_machine_ = low_state.mode_machine();
    }
  }

  void LeftDex1StateHandler(const void* message) {
    const Dex1StateMsg state = *(const Dex1StateMsg*)message;
    if (!state.states().empty()) {
      left_dex1_q_ = state.states()[0].q();
    }
  }

  void RightDex1StateHandler(const void* message) {
    const Dex1StateMsg state = *(const Dex1StateMsg*)message;
    if (!state.states().empty()) {
      right_dex1_q_ = state.states()[0].q();
    }
  }

  array<float, G1_NUM_MOTOR> MakeRaisedArmPose(const array<float, G1_NUM_MOTOR>& start_pose) const {
    array<float, G1_NUM_MOTOR> pose = start_pose;

    pose.at(LeftShoulderPitch) = start_pose.at(LeftShoulderPitch) - 0.80f;
    pose.at(LeftShoulderRoll) = start_pose.at(LeftShoulderRoll) + 0.25f;
    pose.at(LeftShoulderYaw) = start_pose.at(LeftShoulderYaw) + 0.10f;
    pose.at(LeftElbow) = start_pose.at(LeftElbow) - 1.20f;
    pose.at(LeftWristRoll) = start_pose.at(LeftWristRoll);
    pose.at(LeftWristPitch) = start_pose.at(LeftWristPitch);
    pose.at(LeftWristYaw) = start_pose.at(LeftWristYaw);

    pose.at(RightShoulderPitch) = start_pose.at(RightShoulderPitch) - 0.80f;
    pose.at(RightShoulderRoll) = start_pose.at(RightShoulderRoll) - 0.25f;
    pose.at(RightShoulderYaw) = start_pose.at(RightShoulderYaw) - 0.10f;
    pose.at(RightElbow) = start_pose.at(RightElbow) - 1.20f;
    pose.at(RightWristRoll) = start_pose.at(RightWristRoll);
    pose.at(RightWristPitch) = start_pose.at(RightWristPitch);
    pose.at(RightWristYaw) = start_pose.at(RightWristYaw);

    return pose;
  }

  void MoveArmsBetweenPoses(const array<float, G1_NUM_MOTOR>& start_pose,
                            const array<float, G1_NUM_MOTOR>& target_pose,
                            float duration_seconds,
                            bool command_grippers,
                            float left_gripper_q,
                            float right_gripper_q) {
    const int steps = max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    array<float, G1_NUM_MOTOR> command_pose = start_pose;

    for (int step = 0; step <= steps; ++step) {
      const float ratio = SmoothStep(static_cast<float>(step) / static_cast<float>(steps));
      for (int joint = LeftShoulderPitch; joint <= LeftWristYaw; ++joint) {
        command_pose.at(joint) = Lerp(start_pose.at(joint), target_pose.at(joint), ratio);
      }
      for (int joint = RightShoulderPitch; joint <= RightWristYaw; ++joint) {
        command_pose.at(joint) = Lerp(start_pose.at(joint), target_pose.at(joint), ratio);
      }

      PublishG1Pose(command_pose);
      if (command_grippers) {
        PublishBothDex1Grippers(left_gripper_q, right_gripper_q);
      }
      this_thread::sleep_for(chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
    }
  }

  void HoldArmPose(const array<float, G1_NUM_MOTOR>& pose,
                   float duration_seconds,
                   bool command_grippers,
                   float left_gripper_q,
                   float right_gripper_q) {
    const int steps = max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    for (int step = 0; step < steps; ++step) {
      PublishG1Pose(pose);
      if (command_grippers) {
        PublishBothDex1Grippers(left_gripper_q, right_gripper_q);
      }
      this_thread::sleep_for(chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
    }
  }

  void PublishG1Pose(const array<float, G1_NUM_MOTOR>& pose) {
    LowCmd_ command;
    command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    command.mode_machine() = mode_machine_;

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      command.motor_cmd().at(i).mode() = 1;
      command.motor_cmd().at(i).tau() = 0.0f;
      command.motor_cmd().at(i).q() = pose.at(i);
      command.motor_cmd().at(i).dq() = 0.0f;
      command.motor_cmd().at(i).kp() = Kp.at(i);
      command.motor_cmd().at(i).kd() = Kd.at(i);
    }

    command.crc() = Crc32Core((uint32_t*)&command, (sizeof(command) >> 2) - 1);
    lowcmd_publisher_->Write(command);
  }

  Dex1CmdMsg MakeDex1Command(float q) const {
    Dex1CmdMsg command;
    command.cmds().resize(1);
    command.cmds()[0].mode() = 1;
    command.cmds()[0].q() = q;
    command.cmds()[0].dq() = 0.0f;
    command.cmds()[0].tau() = 0.0f;
    command.cmds()[0].kp() = kDex1Kp;
    command.cmds()[0].kd() = kDex1Kd;
    return command;
  }

  void PublishBothDex1Grippers(float left_q, float right_q) {
    left_dex1_cmd_publisher_->Write(MakeDex1Command(left_q));
    right_dex1_cmd_publisher_->Write(MakeDex1Command(right_q));
  }

  atomic<float> left_dex1_q_{0.0f};
  atomic<float> right_dex1_q_{0.0f};
};

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    cout << "Usage: g1_dual_dex1_arm_hand_motion network_interface" << endl;
    cout << "This version controls G1 dual arms with Dex1-1 left/right grippers." << endl;
    return 0;
  }

  const string network_interface = argv[1];
  G1DualArmDex1GripperExample example(network_interface);

  if (!example.WaitForG1State()) {
    return 1;
  }

  example.RunDemoSequence();
  return 0;
}
