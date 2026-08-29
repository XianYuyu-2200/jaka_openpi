#include "jaka_arm_backend.hpp"

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

}  // namespace jaka_mini2
