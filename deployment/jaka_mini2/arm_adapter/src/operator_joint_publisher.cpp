#include "JAKAZuRobot.h"
#include "dry_run_core.hpp"
#include "joint_sample_ipc.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};

void handle_signal(int) {
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

struct SharedOperatorState {
    std::mutex mutex;
    std::condition_variable sample_ready;
    JointValue joints{};
    RobotStatus_simple status{};
    BOOL dragging{FALSE};
    int joint_ret{-3};
    int status_ret{-3};
    int drag_ret{-3};
    std::uint64_t sample_ns{0};
    bool has_sample{false};
    std::uint64_t generation{0};
    std::uint64_t read_ok{0};
    std::uint64_t read_errors{0};
    std::uint64_t max_joint_call_ns{0};
    std::uint64_t max_status_call_ns{0};
    std::uint64_t max_drag_call_ns{0};
};

int env_cpu(const char* name, int fallback) {
    const char* value = std::getenv(name);
    return value == nullptr ? fallback : std::stoi(value);
}

bool pin_current_thread(int cpu) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return pthread_setaffinity_np(
               pthread_self(), sizeof(set), &set) == 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <operator_ip> <socket_path> [duration_sec]\n";
        return 64;
    }

    const char* operator_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 4 ? std::stod(argv[3]) : 0.0;
    const int publisher_cpu = env_cpu("JAKA_PUBLISHER_CPU", 10);
    const int sampler_cpu = env_cpu("JAKA_SAMPLER_CPU", 6);

    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
        std::cerr << "socket path is too long\n";
        return 65;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << '\n';
        return 66;
    }


    sockaddr_un destination{};
    destination.sun_family = AF_UNIX;
    std::strncpy(destination.sun_path,
                 socket_path.c_str(),
                 sizeof(destination.sun_path) - 1);

    JAKAZuRobot robot;
    if (!pin_current_thread(publisher_cpu)) {
        std::cerr << "failed to pin publisher thread to CPU "
                  << publisher_cpu << '\n';
        close(socket_fd);
        return 67;
    }
    const int login_ret = robot.login_in(operator_ip, false);
    std::cout << "operator login_ret=" << login_ret
              << " ip=" << operator_ip
              << " publisher_cpu=" << publisher_cpu
              << " sampler_cpu=" << sampler_cpu << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    constexpr std::uint64_t period_ns = 8'000'000;
    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t sequence = 0;
    std::uint64_t sent = 0;
    std::uint64_t send_errors = 0;
    std::uint64_t last_publish_ns = 0;
    std::uint64_t max_publish_interval_ns = 0;
    std::uint64_t missed_periods = 0;
    std::uint64_t repeated_samples = 0;
    std::uint64_t last_generation = 0;
    std::uint64_t max_sample_age_ns = 0;
    long double sample_age_sum_ns = 0.0;
    std::uint64_t sample_age_observations = 0;
    SharedOperatorState shared;

    // All SDK reads stay on this thread. The publishing loop wakes immediately
    // for a new sample and uses an 8 ms heartbeat if an SDK call stalls.
    std::thread sampling_thread([&]() {
        if (!pin_current_thread(sampler_cpu)) {
            std::cerr << "failed to pin sampling thread to CPU "
                      << sampler_cpu << '\n';
            running.store(false);
            return;
        }
        std::uint64_t sample_index = 0;
        std::uint64_t sample_next_ns = monotonic_ns();
        jaka_dry_run::StatusPollScheduler status_poll(25, 0);
        jaka_dry_run::StatusPollScheduler drag_poll(25, 12);
        RobotStatus_simple cached_status{};
        BOOL cached_dragging = FALSE;
        int cached_status_ret = -3;
        int cached_drag_ret = -3;

        while (running.load()) {
            // Run the slow, low-rate readiness queries before the joint read.
            // This ensures every mailbox timestamp is taken after all such
            // blocking work rather than making a fresh joint sample old.
            std::uint64_t status_call_ns = 0;
            std::uint64_t drag_call_ns = 0;
            if (status_poll.should_poll(sample_index)) {
                const std::uint64_t started = monotonic_ns();
                cached_status_ret = robot.get_robot_status_simple(&cached_status);
                status_call_ns = monotonic_ns() - started;
            }
            if (drag_poll.should_poll(sample_index)) {
                const std::uint64_t started = monotonic_ns();
                cached_drag_ret = robot.is_in_drag_mode(&cached_dragging);
                drag_call_ns = monotonic_ns() - started;
            }
            {
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.status = cached_status;
                shared.dragging = cached_dragging;
                shared.status_ret = cached_status_ret;
                shared.drag_ret = cached_drag_ret;
                shared.max_status_call_ns =
                    std::max(shared.max_status_call_ns, status_call_ns);
                shared.max_drag_call_ns =
                    std::max(shared.max_drag_call_ns, drag_call_ns);
            }

            JointValue joints{};
            const std::uint64_t joint_call_started_ns = monotonic_ns();
            const int joint_ret = robot.get_actual_joint_position(&joints);
            const std::uint64_t joint_call_ns =
                monotonic_ns() - joint_call_started_ns;

            // Publish the joint sample immediately after the SDK read. Do not
            // include the slower status/drag queries in its timestamp.
            const std::uint64_t joint_sample_ns = monotonic_ns();
            {
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.joints = joints;
                shared.joint_ret = joint_ret;
                shared.sample_ns = joint_sample_ns;
                shared.has_sample = true;
                ++shared.generation;
                if (joint_ret == 0) {
                    ++shared.read_ok;
                } else {
                    ++shared.read_errors;
                }
                shared.max_joint_call_ns =
                    std::max(shared.max_joint_call_ns, joint_call_ns);
            }
            shared.sample_ready.notify_one();

            ++sample_index;
            sample_next_ns += period_ns;
            const std::uint64_t after_work_ns = monotonic_ns();
            if (sample_next_ns <= after_work_ns) {
                sample_next_ns = after_work_ns;
            }
            const timespec deadline = ns_to_timespec(sample_next_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        }
    });

    while (running.load()) {
        const std::uint64_t cycle_ns = monotonic_ns();
        if (duration_sec > 0.0 &&
            cycle_ns - started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            break;
        }

        SharedOperatorState snapshot;
        std::uint64_t snapshot_generation = 0;
        {
            std::unique_lock<std::mutex> lock(shared.mutex);
            shared.sample_ready.wait_for(
                lock,
                std::chrono::nanoseconds(period_ns),
                [&]() {
                    return !running.load() ||
                           (shared.has_sample &&
                            shared.generation != last_generation);
                });
            if (!running.load()) break;
            snapshot.joints = shared.joints;
            snapshot.status = shared.status;
            snapshot.dragging = shared.dragging;
            snapshot.joint_ret = shared.joint_ret;
            snapshot.status_ret = shared.status_ret;
            snapshot.drag_ret = shared.drag_ret;
            snapshot.sample_ns = shared.sample_ns;
            snapshot.has_sample = shared.has_sample;
            snapshot_generation = shared.generation;
        }
        if (!snapshot.has_sample) {
            continue;
        }
        if (snapshot_generation == last_generation) {
            ++repeated_samples;
        }
        last_generation = snapshot_generation;
        const std::uint64_t sample_age_ns =
            monotonic_ns() - snapshot.sample_ns;
        max_sample_age_ns = std::max(max_sample_age_ns, sample_age_ns);
        sample_age_sum_ns += sample_age_ns;
        ++sample_age_observations;

        jaka_ipc::JointSamplePacket packet{};
        packet.magic = jaka_ipc::kMagic;
        packet.version = jaka_ipc::kVersion;
        packet.size = sizeof(packet);
        packet.sequence = sequence++;
        packet.monotonic_ns = snapshot.sample_ns;
        const std::uint64_t publish_ns = monotonic_ns();
        if (last_publish_ns != 0) {
            const std::uint64_t publish_interval_ns =
                publish_ns - last_publish_ns;
            max_publish_interval_ns = std::max(
                max_publish_interval_ns,
                publish_interval_ns);
            if (publish_interval_ns >= 2 * period_ns) {
                missed_periods += publish_interval_ns / period_ns - 1;
            }
        }
        last_publish_ns = publish_ns;
        packet.sdk_code = snapshot.joint_ret;
        packet.operator_powered =
            snapshot.status_ret == 0 && snapshot.status.powered_on ? 1 : 0;
        packet.operator_enabled =
            snapshot.status_ret == 0 && snapshot.status.enabled ? 1 : 0;
        packet.operator_dragging =
            snapshot.drag_ret == 0 && snapshot.dragging ? 1 : 0;
        packet.operator_valid =
            snapshot.joint_ret == 0 && snapshot.status_ret == 0 &&
                    snapshot.drag_ret == 0
                ? 1
                : 0;

        if (snapshot.joint_ret == 0) {
            for (int joint = 0; joint < 6; ++joint) {
                packet.position[joint] = snapshot.joints.jVal[joint];
            }
        }

        const ssize_t bytes = sendto(
            socket_fd,
            &packet,
            sizeof(packet),
            0,
            reinterpret_cast<const sockaddr*>(&destination),
            sizeof(destination));
        if (bytes == static_cast<ssize_t>(sizeof(packet))) {
            ++sent;
        } else {
            ++send_errors;
        }

        if (sequence % 125 == 0) {
            const double elapsed =
                static_cast<double>(monotonic_ns() - started_ns) / 1e9;
            std::cout << std::fixed << std::setprecision(2)
                      << "operator sequence=" << sequence
                      << " rate_hz=" << sequence / elapsed
                      << " sent=" << sent
                      << " send_errors=" << send_errors
                      << " powered=" << static_cast<int>(packet.operator_powered)
                      << " enabled=" << static_cast<int>(packet.operator_enabled)
                      << " dragging=" << static_cast<int>(packet.operator_dragging)
                      << '\n';
        }

    }

    running.store(false);
    sampling_thread.join();
    std::uint64_t read_ok = 0;
    std::uint64_t read_errors = 0;
    std::uint64_t max_joint_call_ns = 0;
    std::uint64_t max_status_call_ns = 0;
    std::uint64_t max_drag_call_ns = 0;
    {
        std::lock_guard<std::mutex> lock(shared.mutex);
        read_ok = shared.read_ok;
        read_errors = shared.read_errors;
        max_joint_call_ns = shared.max_joint_call_ns;
        max_status_call_ns = shared.max_status_call_ns;
        max_drag_call_ns = shared.max_drag_call_ns;
    }
    const int logout_ret = robot.login_out();
    close(socket_fd);
    std::cout << "operator summary samples=" << sequence
              << " read_ok=" << read_ok
              << " read_errors=" << read_errors
              << " sent=" << sent
              << " send_errors=" << send_errors
              << " max_publish_interval_ms="
              << std::fixed << std::setprecision(6)
              << static_cast<double>(max_publish_interval_ns) / 1e6
              << " max_operator_joint_call_ms="
              << static_cast<double>(max_joint_call_ns) / 1e6
              << " max_operator_status_call_ms="
              << static_cast<double>(max_status_call_ns) / 1e6
              << " max_operator_drag_call_ms="
              << static_cast<double>(max_drag_call_ns) / 1e6
              << " missed_periods=" << missed_periods
              << " repeated_samples=" << repeated_samples
              << " max_sample_age_ms="
              << static_cast<double>(max_sample_age_ns) / 1e6
              << " avg_sample_age_ms="
              << (sample_age_observations == 0
                      ? 0.0
                      : static_cast<double>(sample_age_sum_ns /
                                            sample_age_observations / 1e6))
              << " logout_ret=" << logout_ret << std::endl;

    return read_ok > 0 && sent > 0 && logout_ret == 0 ? 0 : 2;
}
