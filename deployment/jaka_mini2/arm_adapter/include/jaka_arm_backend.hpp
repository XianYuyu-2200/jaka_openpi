#pragma once

#include "JAKAZuRobot.h"

#include <array>
#include <string>
#include <utility>

namespace jaka_mini2 {

struct ArmState {
    std::array<double, 6> joint_position_rad{};
};

struct TcpPose {
    std::array<double, 3> xyz_mm{};
    std::array<double, 3> rpy_rad{};
};

struct HandState {
    std::array<double, 6> position{};
    bool verified{false};
};

class JakaArmBackend {
public:
    explicit JakaArmBackend(std::string robot_ip, bool allow_motion = false);
    ~JakaArmBackend();

    JakaArmBackend(const JakaArmBackend&) = delete;
    JakaArmBackend& operator=(const JakaArmBackend&) = delete;

    int connect();
    int disconnect();
    int read_state(ArmState* state);
    int forward_kinematics(const ArmState& state, TcpPose* pose);

    // Motion is fail-closed. The constructor must explicitly receive true,
    // and the caller must call arm_motion() before send_servo_j().
    int arm_motion();
    int disarm_motion();
    int send_servo_j(const ArmState& target);
    int stop();
    int read_o6_state(HandState* state);
    int send_o6_position_direct(const HandState& target);

    bool connected() const { return connected_; }
    bool motion_armed() const { return motion_armed_; }

private:
    std::string robot_ip_;
    bool allow_motion_{false};
    bool connected_{false};
    bool motion_armed_{false};
    JAKAZuRobot robot_;
};

}  // namespace jaka_mini2
