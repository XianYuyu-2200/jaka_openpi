#include "jaka_arm_c_api.h"

#include "jaka_arm_backend.hpp"

#include <new>

namespace {

jaka_mini2::JakaArmBackend* backend(void* handle) {
    return static_cast<jaka_mini2::JakaArmBackend*>(handle);
}

}  // namespace

extern "C" {

void* jaka_arm_create(const char* robot_ip, int allow_motion) {
    if (robot_ip == nullptr) return nullptr;
    return new (std::nothrow) jaka_mini2::JakaArmBackend(robot_ip, allow_motion != 0);
}

void jaka_arm_destroy(void* handle) {
    delete backend(handle);
}

int jaka_arm_connect(void* handle) {
    return handle == nullptr ? -1 : backend(handle)->connect();
}

int jaka_arm_disconnect(void* handle) {
    return handle == nullptr ? -1 : backend(handle)->disconnect();
}

int jaka_arm_read_state(void* handle, double joint_position_rad[6]) {
    if (handle == nullptr || joint_position_rad == nullptr) return -1;
    jaka_mini2::ArmState state{};
    const int result = backend(handle)->read_state(&state);
    if (result == 0) {
        for (int index = 0; index < 6; ++index) joint_position_rad[index] = state.joint_position_rad[index];
    }
    return result;
}

int jaka_arm_forward_kinematics(void* handle, const double joint_position_rad[6], double tcp_pose[6]) {
    if (handle == nullptr || joint_position_rad == nullptr || tcp_pose == nullptr) return -1;
    jaka_mini2::ArmState state{};
    for (int index = 0; index < 6; ++index) state.joint_position_rad[index] = joint_position_rad[index];
    jaka_mini2::TcpPose pose{};
    const int result = backend(handle)->forward_kinematics(state, &pose);
    if (result == 0) {
        for (int index = 0; index < 3; ++index) {
            tcp_pose[index] = pose.xyz_mm[index];
            tcp_pose[index + 3] = pose.rpy_rad[index];
        }
    }
    return result;
}

int jaka_arm_arm_motion(void* handle) {
    return handle == nullptr ? -1 : backend(handle)->arm_motion();
}

int jaka_arm_disarm_motion(void* handle) {
    return handle == nullptr ? -1 : backend(handle)->disarm_motion();
}

int jaka_arm_send_servo_j(void* handle, const double joint_position_rad[6]) {
    if (handle == nullptr || joint_position_rad == nullptr) return -1;
    jaka_mini2::ArmState target{};
    for (int index = 0; index < 6; ++index) target.joint_position_rad[index] = joint_position_rad[index];
    return backend(handle)->send_servo_j(target);
}

int jaka_arm_stop(void* handle) {
    return handle == nullptr ? -1 : backend(handle)->stop();
}

int jaka_o6_read_state(void* handle, double hand_position[6]) {
    if (handle == nullptr || hand_position == nullptr) return -1;
    jaka_mini2::HandState state{};
    const int result = backend(handle)->read_o6_state(&state);
    if (result == 0 && state.verified) {
        for (int index = 0; index < 6; ++index) hand_position[index] = state.position[index];
    }
    return result;
}

}  // extern "C"
