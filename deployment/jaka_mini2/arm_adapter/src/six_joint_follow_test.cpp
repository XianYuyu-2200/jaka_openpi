#include "JAKAZuRobot.h"
#include "joint_sample_ipc.hpp"
#include "dry_run_core.hpp"
#include "six_joint_mapping.hpp"
#include "teleop_recording_ipc.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};
constexpr std::uint32_t kO6CommandMagic = 0x364F434A;
constexpr std::uint16_t kO6CommandVersion = 1;

#pragma pack(push, 1)
struct O6CommandPacket {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t size;
    std::uint64_t monotonic_ns;
    std::uint16_t position[6];
};
#pragma pack(pop)

static_assert(sizeof(O6CommandPacket) == 28);

void stop(int) {
    running.store(false);
}

std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(now.tv_nsec);
}

timespec ns_to_timespec(std::uint64_t value) {
    return timespec{
        static_cast<time_t>(value / 1'000'000'000ULL),
        static_cast<long>(value % 1'000'000'000ULL),
    };
}

void print_joint_array(const double values[6],
                       double scale = 1.0,
                       std::ostream& output = std::cout) {
    output << '[';
    for (int joint = 0; joint < 6; ++joint) {
        if (joint != 0) output << ", ";
        output << values[joint] * scale;
    }
    output << ']';
}

int send_o6_position(JAKAZuRobot& robot, const O6CommandPacket& packet) {
    std::array<std::uint8_t, 21> frame{
        0x27, 0x10, 0x00, 0x00, 0x00, 0x06, 0x0C,
    };
    for (int index = 0; index < 6; ++index) {
        if (packet.position[index] > 255) return -2;
        frame[7 + index * 2] = 0x00;
        frame[8 + index * 2] =
            static_cast<std::uint8_t>(packet.position[index]);
    }
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < 19; ++i) {
        crc ^= frame[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) != 0
                      ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001)
                      : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    frame[19] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[20] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    return robot.send_tio_rs_command(
        0, frame.data(), static_cast<int>(frame.size()));
}

int read_o6_position(JAKAZuRobot& robot, std::uint16_t position[6]) {
    std::array<SignInfo, 64> signals{};
    int count = static_cast<int>(signals.size());
    const int result = robot.get_rs485_signal_info(signals.data(), &count);
    if (result != 0 || count < 6 || count > static_cast<int>(signals.size())) {
        return result != 0 ? result : -2;
    }
    constexpr std::array<const char*, 6> names = {
        "o6_r_pos0", "o6_r_pos1", "o6_r_pos2",
        "o6_r_pos3", "o6_r_pos4", "o6_r_pos5"};
    std::array<bool, 6> found{};
    for (int i = 0; i < count; ++i) {
        signals[i].sig_name[sizeof(signals[i].sig_name) - 1] = '\0';
        for (int joint = 0; joint < 6; ++joint) {
            if (std::strcmp(signals[i].sig_name, names[joint]) == 0 &&
                signals[i].chn_id == 0 && signals[i].sig_type == 4 &&
                signals[i].sig_addr == joint && signals[i].value >= 0 &&
                signals[i].value <= 255) {
                position[joint] = static_cast<std::uint16_t>(signals[i].value);
                found[joint] = true;
            }
        }
    }
    for (bool present : found) {
        if (!present) return -3;
    }
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 4 || argc > 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <tracking_ip> <ipc_socket> START_SIX_JOINTS"
                     " [duration_sec]\n";
        return 64;
    }
    if (std::string(argv[3]) != "START_SIX_JOINTS") {
        std::cerr << "refusing to start without START_SIX_JOINTS token\n";
        return 65;
    }

    const char* tracking_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 5 ? std::stod(argv[4]) : 0.0;
    const std::string filter_mode =
        std::getenv("JAKA_SERVO_FILTER") == nullptr
            ? "baseline"
            : std::getenv("JAKA_SERVO_FILTER");
    const double lpf_cutoff =
        std::getenv("JAKA_LPF_CUTOFF") == nullptr
            ? 1.0
            : std::stod(std::getenv("JAKA_LPF_CUTOFF"));
    const bool hold_on_operator_idle =
        std::getenv("JAKA_HOLD_ON_OPERATOR_IDLE") != nullptr &&
        std::string(std::getenv("JAKA_HOLD_ON_OPERATOR_IDLE")) == "1";
    if (!(lpf_cutoff > 0.0 && lpf_cutoff <= 20.0)) {
        std::cerr << "JAKA_LPF_CUTOFF must be in (0, 20]\n";
        return 64;
    }
    unsigned int servo_step_num = 1;
    if (const char* configured = std::getenv("JAKA_SERVO_STEP_NUM")) {
        const int parsed = std::stoi(configured);
        if (parsed < 1 || parsed > 4) {
            std::cerr << "JAKA_SERVO_STEP_NUM must be in 1..4\n";
            return 64;
        }
        servo_step_num = static_cast<unsigned int>(parsed);
    }
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) return 66;

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd < 0) return 67;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path,
                 socket_path.c_str(),
                 sizeof(address.sun_path) - 1);
    unlink(socket_path.c_str());
    if (bind(socket_fd,
             reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
        close(socket_fd);
        return 68;
    }

    JAKAZuRobot robot;
    const int login_ret = robot.login_in(tracking_ip, false);
    std::cout << "six-joint follower login_ret=" << login_ret
              << " ip=" << tracking_ip
              << " servo_step_num=" << servo_step_num
              << " interpolation_ms=" << servo_step_num * 8
              << " hold_on_operator_idle=" << (hold_on_operator_idle ? 1 : 0)
              << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        unlink(socket_path.c_str());
        return 1;
    }

    int filter_ret = 0;
    if (filter_mode == "baseline") {
        filter_ret = robot.servo_move_use_none_filter();
    } else if (filter_mode == "nlf") {
        // Units are deg/s, deg/s^2 and deg/s^3. These limits are deliberately
        // above the host-side limits, so NLF smooths jerk without becoming a
        // second slow velocity controller.
        filter_ret = robot.servo_move_use_joint_NLF(45.0, 600.0, 6000.0);
    } else if (filter_mode == "lpf") {
        filter_ret = robot.servo_move_use_joint_LPF(lpf_cutoff);
    } else {
        std::cerr << "JAKA_SERVO_FILTER must be baseline, nlf or lpf\n";
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 64;
    }
    std::cout << "servo_filter=" << filter_mode
              << (filter_mode == "lpf"
                      ? " lpf_cutoff=" + std::to_string(lpf_cutoff)
                      : "")
              << " configure_ret=" << filter_ret << '\n';
    if (filter_ret != 0) {
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 2;
    }

    RobotStatus_simple tracking_status{};
    const int status_ret = robot.get_robot_status_simple(&tracking_status);
    if (status_ret != 0 || !tracking_status.powered_on ||
        !tracking_status.enabled) {
        std::cerr << "tracking arm must already be powered and enabled; "
                  << "status_ret=" << status_ret
                  << " powered=" << tracking_status.powered_on
                  << " enabled=" << tracking_status.enabled << '\n';
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 2;
    }

    JointValue initial{};
    const int initial_ret = robot.get_actual_joint_position(&initial);
    if (initial_ret != 0) {
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 3;
    }

    int o6_socket_fd = -1;
    std::string o6_socket_path;
    if (const char* configured = std::getenv("JAKA_O6_CONTROL_SOCKET")) {
        o6_socket_path = configured;
        if (o6_socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
            robot.servo_move_use_none_filter();
            robot.login_out();
            close(socket_fd);
            unlink(socket_path.c_str());
            return 69;
        }
        o6_socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (o6_socket_fd < 0) {
            robot.servo_move_use_none_filter();
            robot.login_out();
            close(socket_fd);
            unlink(socket_path.c_str());
            return 70;
        }
        sockaddr_un o6_address{};
        o6_address.sun_family = AF_UNIX;
        std::strncpy(o6_address.sun_path,
                     o6_socket_path.c_str(),
                     sizeof(o6_address.sun_path) - 1);
        unlink(o6_socket_path.c_str());
        if (bind(o6_socket_fd,
                 reinterpret_cast<const sockaddr*>(&o6_address),
                 sizeof(o6_address)) != 0) {
            close(o6_socket_fd);
            robot.servo_move_use_none_filter();
            robot.login_out();
            close(socket_fd);
            unlink(socket_path.c_str());
            return 71;
        }
        std::cout << "o6_control_socket=" << o6_socket_path << '\n';
    }

    int recording_socket_fd = -1;
    std::string recording_socket_path;
    sockaddr_un recording_destination{};
    if (const char* configured = std::getenv("JAKA_RECORD_STATE_SOCKET")) {
        recording_socket_path = configured;
        if (recording_socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
            if (o6_socket_fd >= 0) close(o6_socket_fd);
            if (!o6_socket_path.empty()) unlink(o6_socket_path.c_str());
            robot.servo_move_use_none_filter();
            robot.login_out();
            close(socket_fd);
            unlink(socket_path.c_str());
            return 72;
        }
        recording_socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (recording_socket_fd < 0) {
            if (o6_socket_fd >= 0) close(o6_socket_fd);
            if (!o6_socket_path.empty()) unlink(o6_socket_path.c_str());
            robot.servo_move_use_none_filter();
            robot.login_out();
            close(socket_fd);
            unlink(socket_path.c_str());
            return 73;
        }
        recording_destination.sun_family = AF_UNIX;
        std::strncpy(recording_destination.sun_path,
                     recording_socket_path.c_str(),
                     sizeof(recording_destination.sun_path) - 1);
        std::cout << "record_state_socket=" << recording_socket_path << '\n';
    }

    double tracking_zero[6]{};
    double operator_zero[6]{};
    for (int joint = 0; joint < 6; ++joint) {
        tracking_zero[joint] = initial.jVal[joint];
    }

    bool have_operator_zero = false;
    bool servo_enabled = false;
    std::uint64_t last_packet_ns = 0;
    std::uint64_t last_sequence = 0;
    std::uint64_t commands = 0;
    std::uint64_t missed_servo_periods = 0;
    std::uint64_t max_latency_ns = 0;
    std::uint64_t max_servo_call_ns = 0;
    std::uint64_t max_command_interval_ns = 0;
    std::uint64_t servo_calls_over_8ms = 0;
    std::uint64_t servo_calls_over_16ms = 0;
    std::uint64_t servo_calls_over_24ms = 0;
    std::uint64_t last_command_started_ns = 0;
    std::uint64_t o6_commands = 0;
    std::uint64_t o6_failures = 0;
    std::uint64_t max_o6_call_ns = 0;
    std::uint64_t recording_samples = 0;
    std::uint64_t recording_send_errors = 0;
    std::uint64_t max_recording_read_ns = 0;
    long double latency_sum_ns = 0.0;
    const std::uint64_t launch_ns = monotonic_ns();
    std::uint64_t active_started_ns = 0;
    std::uint64_t active_ended_ns = 0;
    std::string stop_reason="PROCESS_EXIT";

    // IPC reception is decoupled from servo output.  JAKA interpolates every
    // 8 ms; sending at that fixed cadence avoids packet-arrival jitter being
    // turned directly into motion jitter.
    constexpr std::uint64_t servo_period_ns = 8'000'000;
    std::uint64_t next_servo_ns = monotonic_ns();
    jaka_ipc::JointSamplePacket latest_packet{};
    std::uint64_t latest_received_ns = 0;
    bool have_latest_packet = false;
    bool operator_ready_now = false;
    std::uint64_t operator_not_ready_since_ns = 0;
    jaka_dry_run::TargetSmoother smoother;
    double operator_position[6]{};
    double desired_target[6]{};
    O6CommandPacket pending_o6{};
    bool have_pending_o6 = false;
    std::array<std::uint16_t, 6> current_o6_action{};
    bool have_o6_action = false;
    std::array<std::uint16_t, 6> recording_actual_hand{};
    int recording_hand_state_ret = -4;

    while (running.load()) {
        const std::uint64_t loop_now_ns = monotonic_ns();
        if (active_started_ns != 0 && duration_sec > 0.0 &&
            loop_now_ns - active_started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            stop_reason = "DURATION";
            break;
        }
        if (!hold_on_operator_idle && !have_operator_zero &&
            loop_now_ns - launch_ns >= 15'000'000'000ULL) {
            stop_reason = "OPERATOR_READINESS_TIMEOUT";
            break;
        }

        // Drain the datagram socket and retain only the newest valid sample.
        while (true) {
            jaka_ipc::JointSamplePacket candidate{};
            const ssize_t bytes = recv(socket_fd, &candidate, sizeof(candidate),
                                       MSG_DONTWAIT);
            if (bytes < 0) break;
            const std::uint64_t received_ns = monotonic_ns();
            if (bytes != static_cast<ssize_t>(sizeof(candidate)) ||
                !jaka_ipc::valid_packet(candidate) || candidate.sdk_code != 0) {
                stop_reason = "INVALID_OPERATOR_PACKET";
                running.store(false);
                break;
            }
            if (have_latest_packet && candidate.sequence <= latest_packet.sequence) {
                continue;
            }
            latest_packet = candidate;
            latest_received_ns = received_ns;
            have_latest_packet = true;
            last_packet_ns = received_ns;
        }
        if (!running.load()) break;

        if (o6_socket_fd >= 0) {
            while (true) {
                O6CommandPacket candidate{};
                const ssize_t bytes = recv(
                    o6_socket_fd, &candidate, sizeof(candidate), MSG_DONTWAIT);
                if (bytes < 0) break;
                if (bytes != static_cast<ssize_t>(sizeof(candidate)) ||
                    candidate.magic != kO6CommandMagic ||
                    candidate.version != kO6CommandVersion ||
                    candidate.size != sizeof(candidate)) {
                    ++o6_failures;
                    continue;
                }
                pending_o6 = candidate;
                have_pending_o6 = true;
            }
        }

        if (!have_latest_packet) {
            if (last_packet_ns != 0 && loop_now_ns - last_packet_ns >= 100'000'000ULL) {
                stop_reason = "DATA_TIMEOUT";
                break;
            }
        } else {
            const auto& packet = latest_packet;
            const bool operator_ready =
                packet.operator_powered != 0 && packet.operator_enabled != 0 &&
                packet.operator_dragging != 0;
            operator_ready_now = operator_ready;
            if (!operator_ready && !have_operator_zero) {
                // Keep waiting for the operator to enter drag mode.
            } else if (!operator_ready) {
                if (hold_on_operator_idle) {
                    // Practice mode: release the operator and hold the last
                    // target while waiting for the next drag interval. A
                    // dead publisher still trips the independent data-timeout
                    // watchdog below.
                    operator_not_ready_since_ns = 0;
                } else {
                    if (operator_not_ready_since_ns == 0) {
                        operator_not_ready_since_ns = loop_now_ns;
                    } else if (loop_now_ns - operator_not_ready_since_ns >=
                               300'000'000ULL) {
                        stop_reason = "OPERATOR_NOT_READY";
                        break;
                    }
                }
            } else if (!have_operator_zero) {
                operator_not_ready_since_ns = 0;
                for (int joint = 0; joint < 6; ++joint) {
                    operator_zero[joint] = packet.position[joint];
                    operator_position[joint] = packet.position[joint];
                }
                have_operator_zero = true;
                last_sequence = packet.sequence;
                std::cout << "operator zero captured sequence="
                          << packet.sequence << '\n';
                const int servo_enable_ret = robot.servo_move_enable(TRUE);
                std::cout << "servo_move_enable_ret="
                          << servo_enable_ret << std::endl;
                if (servo_enable_ret != 0) {
                    stop_reason = "SERVO_ENABLE_FAILED";
                    break;
                }
                servo_enabled = true;
                active_started_ns = monotonic_ns();
                smoother.reset(jaka_dry_run::JointArray{
                    tracking_zero[0], tracking_zero[1], tracking_zero[2],
                    tracking_zero[3], tracking_zero[4], tracking_zero[5]});
                for (int joint = 0; joint < 6; ++joint) {
                    desired_target[joint] = tracking_zero[joint];
                }
            } else if (packet.sequence > last_sequence) {
                operator_not_ready_since_ns = 0;
                last_sequence = packet.sequence;
                for (int joint = 0; joint < 6; ++joint) {
                    operator_position[joint] = packet.position[joint];
                }
                const auto mapped_target = jaka_six_joint::map_relative(
                    operator_position, operator_zero, tracking_zero);
                for (int joint = 0; joint < 6; ++joint) {
                    desired_target[joint] = mapped_target[joint];
                }
            }
        }

        if (!have_operator_zero || !servo_enabled) {
            next_servo_ns += servo_period_ns;
            const timespec deadline = ns_to_timespec(next_servo_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
            continue;
        }

        const std::uint64_t now_ns = monotonic_ns();
        const std::uint64_t receive_age_ns = now_ns - last_packet_ns;
        const std::uint64_t source_age_ns =
            now_ns >= latest_packet.monotonic_ns
                ? now_ns - latest_packet.monotonic_ns
                : 0;
        if (last_packet_ns != 0 &&
            (receive_age_ns >= 100'000'000ULL ||
             source_age_ns >= 100'000'000ULL)) {
            stop_reason = "DATA_TIMEOUT";
            break;
        }
        const auto smooth = receive_age_ns >= 40'000'000ULL ||
                                    source_age_ns >= 40'000'000ULL ||
                                    !operator_ready_now
                                ? smoother.value()
                                : smoother.update(jaka_dry_run::JointArray{
                                      desired_target[0], desired_target[1],
                                      desired_target[2], desired_target[3],
                                      desired_target[4], desired_target[5]});
        JointValue command{};
        double target_position[6]{};
        for (int joint = 0; joint < 6; ++joint) {
            command.jVal[joint] = smooth[joint];
            target_position[joint] = smooth[joint];
        }
        const std::uint64_t command_started_ns = monotonic_ns();
        if (last_command_started_ns != 0) {
            max_command_interval_ns = std::max(
                max_command_interval_ns,
                command_started_ns - last_command_started_ns);
        }
        last_command_started_ns = command_started_ns;
        // Commands are still refreshed every native 8 ms cycle. A longer
        // controller interpolation horizon bridges the occasional 20-30 ms
        // blocking SDK call without lowering the commanded joint velocity.
        const int command_ret =
            robot.servo_j(&command, MoveMode::ABS, servo_step_num);
        const std::uint64_t servo_call_ns = monotonic_ns() - command_started_ns;
        max_servo_call_ns = std::max(max_servo_call_ns, servo_call_ns);
        if (servo_call_ns > 8'000'000ULL) ++servo_calls_over_8ms;
        if (servo_call_ns > 16'000'000ULL) ++servo_calls_over_16ms;
        if (servo_call_ns > 24'000'000ULL) ++servo_calls_over_24ms;
        if (command_ret != 0) {
            std::cerr << "servo_j failed ret=" << command_ret
                      << " command=";
            print_joint_array(target_position, 1.0, std::cerr);
            std::cerr << '\n';
            stop_reason = "SERVO_J_FAILED";
            break;
        }
        ++commands;

        if (have_pending_o6) {
            const std::uint64_t o6_started_ns = monotonic_ns();
            const int o6_ret = send_o6_position(robot, pending_o6);
            const std::uint64_t o6_call_ns = monotonic_ns() - o6_started_ns;
            max_o6_call_ns = std::max(max_o6_call_ns, o6_call_ns);
            ++o6_commands;
            if (o6_ret != 0) ++o6_failures;
            if (o6_ret == 0) {
                for (int index = 0; index < 6; ++index) {
                    current_o6_action[index] = pending_o6.position[index];
                }
                have_o6_action = true;
            }
            std::cout << "o6_command=" << o6_commands
                      << " ret=" << o6_ret
                      << " call_ms="
                      << static_cast<double>(o6_call_ns) / 1e6
                      << " target=[";
            for (int index = 0; index < 6; ++index) {
                if (index != 0) std::cout << ", ";
                std::cout << pending_o6.position[index];
            }
            std::cout << "]\n";
            have_pending_o6 = false;
        }

        // Stagger O6 and arm state reads by about 100 ms so two blocking SDK
        // queries never land in the same 8 ms servo cycle.
        if (recording_socket_fd >= 0 && commands % 25 == 12) {
            const std::uint64_t read_started_ns = monotonic_ns();
            recording_hand_state_ret =
                read_o6_position(robot, recording_actual_hand.data());
            max_recording_read_ns = std::max(
                max_recording_read_ns, monotonic_ns() - read_started_ns);
            if (recording_hand_state_ret == 0 && !have_o6_action) {
                current_o6_action = recording_actual_hand;
                have_o6_action = true;
            }
        }

        if (recording_socket_fd >= 0 && commands % 25 == 0) {
            const std::uint64_t read_started_ns = monotonic_ns();
            JointValue actual{};
            const int arm_state_ret = robot.get_actual_joint_position(&actual);
            const std::uint64_t recording_read_ns =
                monotonic_ns() - read_started_ns;
            max_recording_read_ns =
                std::max(max_recording_read_ns, recording_read_ns);

            jaka_recording_ipc::TeleopSnapshotPacket snapshot{};
            snapshot.magic = jaka_recording_ipc::kMagic;
            snapshot.version = jaka_recording_ipc::kVersion;
            snapshot.size = sizeof(snapshot);
            snapshot.sequence = recording_samples;
            snapshot.monotonic_ns = monotonic_ns();
            snapshot.arm_state_ret = arm_state_ret;
            snapshot.hand_state_ret = recording_hand_state_ret;
            snapshot.operator_dragging = operator_ready_now ? 1 : 0;
            for (int index = 0; index < 6; ++index) {
                snapshot.actual_arm[index] = actual.jVal[index];
                snapshot.action_arm[index] = target_position[index];
                snapshot.actual_hand[index] = recording_actual_hand[index];
                snapshot.action_hand[index] = current_o6_action[index];
            }
            const ssize_t sent_bytes = sendto(
                recording_socket_fd,
                &snapshot,
                sizeof(snapshot),
                0,
                reinterpret_cast<const sockaddr*>(&recording_destination),
                sizeof(recording_destination));
            ++recording_samples;
            if (sent_bytes != static_cast<ssize_t>(sizeof(snapshot))) {
                ++recording_send_errors;
            }
        }

        const std::uint64_t latency_ns =
            latest_received_ns >= latest_packet.monotonic_ns
                ? latest_received_ns - latest_packet.monotonic_ns
                : 0;
        latency_sum_ns += latency_ns;
        if (latency_ns > max_latency_ns) max_latency_ns = latency_ns;

        // Do not call a read API from the real-time command loop.  Those SDK
        // calls can block for multiple interpolation periods.
        if (commands % 625 == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "seq=" << latest_packet.sequence
                      << " operator_q=";
            print_joint_array(operator_position);
            std::cout << " target_q=";
            print_joint_array(target_position);
            std::cout << " latency_ms="
                      << static_cast<double>(latency_ns) / 1e6
                      << " packet_age_ms="
                      << static_cast<double>(receive_age_ns) / 1e6
                      << " source_age_ms="
                      << static_cast<double>(source_age_ns) / 1e6
                      << " servo_call_ms="
                      << static_cast<double>(servo_call_ns) / 1e6
                      << '\n';
        }

        next_servo_ns += servo_period_ns;
        const std::uint64_t after_work_ns = monotonic_ns();
        // If a deadline is only slightly late, issue the next command
        // immediately instead of skipping a complete 8 ms cycle.  Only count
        // and collapse full periods lost to a real SDK/host stall.
        if (after_work_ns > next_servo_ns) {
            missed_servo_periods +=
                (after_work_ns - next_servo_ns) / servo_period_ns;
            next_servo_ns = after_work_ns;
        }
        const timespec deadline = ns_to_timespec(next_servo_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }

    if (!running.load() && stop_reason == "PROCESS_EXIT") {
        stop_reason = "SIGNAL";
    }
    active_ended_ns = monotonic_ns();

    int abort_ret = 0;
    int servo_disable_ret = 0;
    if (servo_enabled) {
        abort_ret = robot.motion_abort();
        servo_disable_ret = robot.servo_move_enable(FALSE);
    }
    const int filter_reset_ret = robot.servo_move_use_none_filter();
    const int logout_ret = robot.login_out();
    if (o6_socket_fd >= 0) close(o6_socket_fd);
    if (recording_socket_fd >= 0) close(recording_socket_fd);
    close(socket_fd);
    if (!o6_socket_path.empty()) unlink(o6_socket_path.c_str());
    unlink(socket_path.c_str());

    std::cout << std::fixed << std::setprecision(6)
              << "six-joint follow summary commands=" << commands
              << " stop_reason=" << stop_reason
              << " active_duration_sec="
              << (active_started_ns == 0
                      ? 0.0
                      : static_cast<double>(active_ended_ns -
                                            active_started_ns) /
                            1e9)
              << " avg_latency_ms="
              << (commands == 0
                      ? 0.0
                      : static_cast<double>(latency_sum_ns / commands / 1e6))
              << " max_latency_ms="
              << static_cast<double>(max_latency_ns) / 1e6
              << " missed_servo_periods=" << missed_servo_periods
              << " max_servo_call_ms="
              << static_cast<double>(max_servo_call_ns) / 1e6
              << " max_command_interval_ms="
              << static_cast<double>(max_command_interval_ns) / 1e6
              << " servo_calls_over_8ms=" << servo_calls_over_8ms
              << " servo_calls_over_16ms=" << servo_calls_over_16ms
              << " servo_calls_over_24ms=" << servo_calls_over_24ms
              << " servo_step_num=" << servo_step_num
              << " abort_ret=" << abort_ret
              << " servo_disable_ret=" << servo_disable_ret
              << " filter_reset_ret=" << filter_reset_ret
              << " o6_commands=" << o6_commands
              << " o6_failures=" << o6_failures
              << " max_o6_call_ms="
              << static_cast<double>(max_o6_call_ns) / 1e6
              << " recording_samples=" << recording_samples
              << " recording_send_errors=" << recording_send_errors
              << " max_recording_read_ms="
              << static_cast<double>(max_recording_read_ns) / 1e6
              << " logout_ret=" << logout_ret << '\n';

    return commands > 0 && abort_ret == 0 && servo_disable_ret == 0 &&
                   filter_reset_ret == 0 && logout_ret == 0
               ? 0
               : 5;
}
