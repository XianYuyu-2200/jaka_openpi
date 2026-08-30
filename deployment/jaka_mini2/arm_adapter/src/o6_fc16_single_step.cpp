#include "JAKAZuRobot.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr int kChannel = 0;  // TIO RS485-1 / RS485H
constexpr int kSlave = 39;   // right O6 (0x27)
constexpr char kToken[] = "ENABLE_O6_FC16_SINGLE_STEP";

std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) != 0 ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001)
                                  : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

std::array<std::uint8_t, 11> make_fc16_single(int address, int value) {
    // slave, function 0x10, address, quantity=1, byte_count=2, value, CRC16 LE
    std::array<std::uint8_t, 11> frame{
        static_cast<std::uint8_t>(kSlave), 0x10,
        static_cast<std::uint8_t>((address >> 8) & 0xFF), static_cast<std::uint8_t>(address & 0xFF),
        0x00, 0x01, 0x02,
        static_cast<std::uint8_t>((value >> 8) & 0xFF), static_cast<std::uint8_t>(value & 0xFF),
        0x00, 0x00};
    const std::uint16_t crc = crc16(frame.data(), 9);
    frame[9] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[10] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    return frame;
}

bool read_positions(JAKAZuRobot& robot, std::array<int, 6>* values) {
    std::array<SignInfo, 64> signals{};
    int count = static_cast<int>(signals.size());
    if (robot.get_rs485_signal_info(signals.data(), &count) != 0) return false;
    constexpr std::array<const char*, 6> names = {
        "o6_r_pos0", "o6_r_pos1", "o6_r_pos2", "o6_r_pos3", "o6_r_pos4", "o6_r_pos5"};
    std::array<bool, 6> found{};
    for (int i = 0; i < count; ++i) {
        signals[i].sig_name[sizeof(signals[i].sig_name) - 1] = '\0';
        for (int j = 0; j < 6; ++j) {
            if (std::strcmp(signals[i].sig_name, names[j]) == 0 && signals[i].chn_id == kChannel &&
                signals[i].sig_type == 4 && signals[i].sig_addr == j && signals[i].value >= 0 &&
                signals[i].value <= 255) {
                (*values)[j] = signals[i].value;
                found[j] = true;
            }
        }
    }
    for (bool present : found) if (!present) return false;
    return true;
}

void print_frame(const std::array<std::uint8_t, 11>& frame) {
    std::cout << "frame=" << std::hex << std::setfill('0');
    for (std::uint8_t byte : frame) std::cout << ' ' << std::setw(2) << static_cast<int>(byte);
    std::cout << std::dec << std::setfill(' ') << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 5 || std::string(argv[4]) != kToken) {
        std::cerr << "Usage: " << argv[0]
                  << " <robot_ip> <channel_index_0_to_5> <delta_-10_to_10> " << kToken << "\n";
        return 64;
    }
    const int index = std::stoi(argv[2]);
    const int delta = std::stoi(argv[3]);
    if (index < 0 || index >= 6 || delta == 0 || delta < -10 || delta > 10) {
        std::cerr << "index must be 0..5 and non-zero delta must be within -10..10\n";
        return 65;
    }

    JAKAZuRobot robot;
    const int login = robot.login_in(argv[1], false);
    if (login != 0) {
        std::cerr << "login failed: " << login << '\n';
        return 1;
    }
    std::array<int, 6> before{};
    if (!read_positions(robot, &before)) {
        std::cerr << "could not read verified o6_r_pos0..o6_r_pos5\n";
        robot.login_out();
        return 2;
    }
    const int target = before[index] + delta;
    if (target < 0 || target > 255) {
        std::cerr << "target outside O6 range: " << target << '\n';
        robot.login_out();
        return 3;
    }

    std::cout << "before=";
    for (int value : before) std::cout << value << ' ';
    std::cout << "channel=" << index << " target=" << target << '\n';
    const auto forward = make_fc16_single(index, target);
    print_frame(forward);
    const int write_result = robot.send_tio_rs_command(kChannel, const_cast<std::uint8_t*>(forward.data()), forward.size());
    if (write_result != 0) {
        std::cerr << "FC16 write failed: " << write_result << '\n';
        robot.login_out();
        return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::array<int, 6> after{};
    const bool read_after = read_positions(robot, &after);
    std::cout << "after_write=";
    if (read_after) for (int value : after) std::cout << value << ' ';
    std::cout << '\n';

    const auto restore = make_fc16_single(index, before[index]);
    print_frame(restore);
    const int restore_result = robot.send_tio_rs_command(kChannel, const_cast<std::uint8_t*>(restore.data()), restore.size());
    std::array<int, 6> final{};
    bool read_final = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        read_final = read_positions(robot, &final);
        if (read_final && final[index] == before[index]) break;
    }
    std::cout << "restored=" << (restore_result == 0 && read_final && final[index] == before[index] ? "yes" : "no") << '\n';
    if (read_final) {
        std::cout << "final=";
        for (int value : final) std::cout << value << ' ';
        std::cout << '\n';
    }
    robot.login_out();
    return write_result == 0 && restore_result == 0 && read_final && final[index] == before[index] ? 0 : 5;
}
