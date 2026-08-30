#include "JAKAZuRobot.h"
#include "joint_sample_ipc.hpp"
#include "dry_run_core.hpp"
#include "six_joint_mapping.hpp"

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

    while (running.load()) {
        const std::uint64_t loop_now_ns = monotonic_ns();
        if (active_started_ns != 0 && duration_sec > 0.0 &&
            loop_now_ns - active_started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            stop_reason = "DURATION";
            break;
        }
        if (!have_operator_zero &&
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
                if (operator_not_ready_since_ns == 0) {
                    operator_not_ready_since_ns = loop_now_ns;
                } else if (loop_now_ns - operator_not_ready_since_ns >=
                           300'000'000ULL) {
                    stop_reason = "OPERATOR_NOT_READY";
                    break;
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
    close(socket_fd);
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
              << " logout_ret=" << logout_ret << '\n';

    return commands > 0 && abort_ret == 0 && servo_disable_ret == 0 &&
                   filter_reset_ret == 0 && logout_ret == 0
               ? 0
               : 5;
}
