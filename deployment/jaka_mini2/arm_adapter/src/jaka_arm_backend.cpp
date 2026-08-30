#include "jaka_arm_backend.hpp"

#include <array>
#include <cstring>

namespace jaka_mini2 {

JakaArmBackend::JakaArmBackend(std::string robot_ip, bool allow_motion)
    : robot_ip_(std::move(robot_ip)), allow_motion_(allow_motion) {}

JakaArmBackend::~JakaArmBackend() {
    if (connected_) disconnect();
}

int JakaArmBackend::connect() {
    if (connected_) return 0;
    const int result = robot_.login_in(robot_ip_.c_str(), false);
    connected_ = result == 0;
    return result;
}

int JakaArmBackend::disconnect() {
    if (!connected_) return 0;
    motion_armed_ = false;
    const int result = robot_.login_out();
    connected_ = false;
    return result;
}

int JakaArmBackend::read_state(ArmState* state) {
    if (!connected_ || state == nullptr) return -1;
    JointValue joints{};
    const int result = robot_.get_actual_joint_position(&joints);
    if (result != 0) return result;
    for (int index = 0; index < 6; ++index) state->joint_position_rad[index] = joints.jVal[index];
    return 0;
}

int JakaArmBackend::forward_kinematics(const ArmState& state, TcpPose* pose) {
    if (!connected_ || pose == nullptr) return -1;
    JointValue joints{};
    for (int index = 0; index < 6; ++index) joints.jVal[index] = state.joint_position_rad[index];
    CartesianPose cartesian{};
    const int result = robot_.kine_forward(&joints, &cartesian);
    if (result != 0) return result;
    pose->xyz_mm = {cartesian.tran.x, cartesian.tran.y, cartesian.tran.z};
    pose->rpy_rad = {cartesian.rpy.rx, cartesian.rpy.ry, cartesian.rpy.rz};
    return 0;
}

int JakaArmBackend::arm_motion() {
    if (!connected_ || !allow_motion_ || motion_armed_) return -1;
    const int result = robot_.servo_move_enable(TRUE);
    motion_armed_ = result == 0;
    return result;
}

int JakaArmBackend::disarm_motion() {
    if (!connected_ || !motion_armed_) return 0;
    const int abort_result = robot_.motion_abort();
    const int disable_result = robot_.servo_move_enable(FALSE);
    motion_armed_ = false;
    return abort_result != 0 ? abort_result : disable_result;
}

int JakaArmBackend::send_servo_j(const ArmState& target) {
    if (!connected_ || !allow_motion_ || !motion_armed_) return -1;
    JointValue joints{};
    for (int index = 0; index < 6; ++index) joints.jVal[index] = target.joint_position_rad[index];
    return robot_.servo_j(&joints, MoveMode::ABS, 1);
}

int JakaArmBackend::stop() {
    return disarm_motion();
}

int JakaArmBackend::read_o6_state(HandState* state) {
    if (!connected_ || state == nullptr) return -1;
    std::array<SignInfo, 64> signals{};
    int count = static_cast<int>(signals.size());
    const int result = robot_.get_rs485_signal_info(signals.data(), &count);
    if (result != 0 || count < 6 || count > static_cast<int>(signals.size())) return result != 0 ? result : -2;
    constexpr std::array<const char*, 6> names = {"o6_r_pos0", "o6_r_pos1", "o6_r_pos2", "o6_r_pos3", "o6_r_pos4", "o6_r_pos5"};
    std::array<bool, 6> found{};
    for (int i = 0; i < count; ++i) {
        signals[i].sig_name[sizeof(signals[i].sig_name) - 1] = '\0';
        for (int j = 0; j < 6; ++j) {
            if (std::strcmp(signals[i].sig_name, names[j]) == 0 && signals[i].chn_id == 0 && signals[i].sig_type == 4 && signals[i].sig_addr == j) {
                if (signals[i].value < 0 || signals[i].value > 255) return -3;
                state->position[j] = static_cast<double>(signals[i].value);
                found[j] = true;
            }
        }
    }
    for (bool present : found) if (!present) return -4;
    state->verified = true;
    return 0;
}

}  // namespace jaka_mini2
