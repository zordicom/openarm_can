// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <openarm/damiao_motor/dm_motor_device_collection.hpp>

namespace openarm::damiao_motor {

DMDeviceCollection::DMDeviceCollection(canbus::CANSocket& can_socket)
    : can_socket_(can_socket),
      can_packet_encoder_(std::make_unique<CanPacketEncoder>()),
      can_packet_decoder_(std::make_unique<CanPacketDecoder>()),
      device_collection_(std::make_unique<canbus::CANDeviceCollection>(can_socket_)) {}

void DMDeviceCollection::enable_all() {
    for (auto dm_device : get_dm_devices()) {
        auto& motor = dm_device->get_motor();
        CANPacket enable_packet = CanPacketEncoder::create_enable_command(motor);
        send_command_to_device(dm_device, enable_packet);
    }
}

void DMDeviceCollection::disable_all() {
    for (auto dm_device : get_dm_devices()) {
        CANPacket disable_packet = CanPacketEncoder::create_disable_command(dm_device->get_motor());
        send_command_to_device(dm_device, disable_packet);
    }
}

void DMDeviceCollection::set_zero(int i) {
    auto dm_device = get_dm_devices().at(i);
    auto zero_packet = CanPacketEncoder::create_set_zero_command(dm_device->get_motor());
    send_command_to_device(dm_device, zero_packet);
}

void DMDeviceCollection::set_zero_all() {
    for (auto dm_device : get_dm_devices()) {
        CANPacket zero_packet = CanPacketEncoder::create_set_zero_command(dm_device->get_motor());
        send_command_to_device(dm_device, zero_packet);
    }
}

void DMDeviceCollection::refresh_one(int i) {
    auto dm_device = get_dm_devices().at(i);
    auto& motor = dm_device->get_motor();
    CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
    send_command_to_device(dm_device, refresh_packet);
    log_command(i, "REFRESH", nullptr, nullptr);
}

void DMDeviceCollection::refresh_all() {
    for (auto dm_device : get_dm_devices()) {
        auto& motor = dm_device->get_motor();
        CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
        send_command_to_device(dm_device, refresh_packet);
    }
}

void DMDeviceCollection::set_callback_mode_all(CallbackMode callback_mode) {
    for (auto dm_device : get_dm_devices()) {
        dm_device->set_callback_mode(callback_mode);
    }
}

void DMDeviceCollection::query_param_one(int i, int RID) {
    CANPacket param_query =
        CanPacketEncoder::create_query_param_command(get_dm_devices()[i]->get_motor(), RID);
    send_command_to_device(get_dm_devices()[i], param_query);
}

void DMDeviceCollection::query_param_all(int RID) {
    for (auto dm_device : get_dm_devices()) {
        CANPacket param_query =
            CanPacketEncoder::create_query_param_command(dm_device->get_motor(), RID);
        send_command_to_device(dm_device, param_query);
    }
}

void DMDeviceCollection::write_param_one(int i, int RID, uint32_t value) {
    CANPacket param_write =
        CanPacketEncoder::create_write_param_command(get_dm_devices()[i]->get_motor(), RID, value);
    send_command_to_device(get_dm_devices()[i], param_write);
}

void DMDeviceCollection::write_param_all(int RID, uint32_t value) {
    for (auto dm_device : get_dm_devices()) {
        CANPacket param_write =
            CanPacketEncoder::create_write_param_command(dm_device->get_motor(), RID, value);
        send_command_to_device(dm_device, param_write);
    }
}

void DMDeviceCollection::send_command_to_device(std::shared_ptr<DMCANDevice> dm_device,
                                                const CANPacket& packet) {
    bool write_success = false;
    write_total_count_++;

    if (can_socket_.is_canfd_enabled()) {
        canfd_frame frame = dm_device->create_canfd_frame(packet.send_can_id, packet.data);
        write_success = can_socket_.write_canfd_frame(frame);
        if (!write_success) {
            write_failure_count_++;
            // Throttle error logging to once per 5 seconds
            static auto last_error_time = std::chrono::steady_clock::time_point::min();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_error_time).count() >= 5) {
                std::cerr << "ERROR: Failed to write CANFD frame to CAN ID 0x"
                          << std::hex << packet.send_can_id << std::dec
                          << " (errno: " << errno << " - " << strerror(errno) << ")"
                          << " [failures: " << write_failure_count_
                          << "/" << write_total_count_ << "]" << std::endl;
                last_error_time = now;
            }
        }
    } else {
        can_frame frame = dm_device->create_can_frame(packet.send_can_id, packet.data);
        write_success = can_socket_.write_can_frame(frame);
        if (!write_success) {
            write_failure_count_++;
            // Throttle error logging to once per 5 seconds
            static auto last_error_time = std::chrono::steady_clock::time_point::min();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_error_time).count() >= 5) {
                std::cerr << "ERROR: Failed to write CAN frame to CAN ID 0x"
                          << std::hex << packet.send_can_id << std::dec
                          << " (errno: " << errno << " - " << strerror(errno) << ")"
                          << " [failures: " << write_failure_count_
                          << "/" << write_total_count_ << "]" << std::endl;
                last_error_time = now;
            }
        }
    }

    // Warn if failure rate exceeds 10%
    if (write_total_count_ % 100 == 0 && write_failure_count_ > write_total_count_ / 10) {
        std::cerr << "WARNING: High CAN write failure rate detected: "
                  << write_failure_count_ << "/" << write_total_count_
                  << " (" << (100.0 * write_failure_count_ / write_total_count_) << "%)"
                  << " - possible bus contention or interface issues" << std::endl;
    }
}

void DMDeviceCollection::mit_control_one(int i, const MITParam& mit_param) {
    CANPacket mit_cmd =
        CanPacketEncoder::create_mit_control_command(get_dm_devices()[i]->get_motor(), mit_param);
    send_command_to_device(get_dm_devices()[i], mit_cmd);
    log_command(i, "MIT", &mit_param, nullptr);
}

void DMDeviceCollection::mit_control_all(const std::vector<MITParam>& mit_params) {
    for (size_t i = 0; i < mit_params.size(); i++) {
        mit_control_one(i, mit_params[i]);
    }
}

std::vector<Motor> DMDeviceCollection::get_motors() const {
    std::vector<Motor> motors;
    for (auto dm_device : get_dm_devices()) {
        motors.push_back(dm_device->get_motor());
    }
    return motors;
}

Motor DMDeviceCollection::get_motor(int i) const { return get_dm_devices().at(i)->get_motor(); }

std::vector<std::shared_ptr<DMCANDevice>> DMDeviceCollection::get_dm_devices() const {
    std::vector<std::shared_ptr<DMCANDevice>> dm_devices;
    for (const auto& [id, device] : device_collection_->get_devices()) {
        auto dm_device = std::dynamic_pointer_cast<DMCANDevice>(device);
        if (dm_device) {
            dm_devices.push_back(dm_device);
        }
    }
    return dm_devices;
}

void DMDeviceCollection::enable_csv_logging(const std::string& log_directory,
                                            const std::string& name_prefix) {
    if (csv_logging_enabled_) {
        std::cerr << "CSV logging already enabled" << std::endl;
        return;
    }

    // Determine log directory: use provided path, or USER_WS if empty
    std::string actual_log_dir;
    if (log_directory.empty()) {
        // Get user workspace directory from environment
        const char* user_ws = std::getenv("USER_WS");
        if (!user_ws) {
            std::cerr << "ERROR: USER_WS environment variable not set and no log directory provided. "
                      << "Cannot create log directory." << std::endl;
            return;
        }
        actual_log_dir = std::string(user_ws) + "/openarm_can_logs";
        std::cout << "Using USER_WS log directory: " << actual_log_dir << std::endl;
    } else {
        actual_log_dir = log_directory;
    }

    // Create directory if it doesn't exist
    std::filesystem::create_directories(actual_log_dir);

    // Get timestamp with millisecond precision
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    int64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();

    std::string log_filename = actual_log_dir + "/" + name_prefix + "_" +
                               std::to_string(timestamp_ms) + ".csv";

    csv_log_file_.open(log_filename, std::ios::out);
    if (!csv_log_file_.is_open()) {
        std::cerr << "Failed to open CSV log file: " << log_filename << std::endl;
        return;
    }

    csv_logging_enabled_ = true;
    csv_log_start_time_ = std::chrono::steady_clock::now();
    csv_flush_counter_ = 0;

    // Write CSV header
    csv_log_file_ << "timestamp";

    auto num_motors = get_dm_devices().size();

    // For each motor, add command columns
    for (size_t i = 0; i < num_motors; ++i) {
        csv_log_file_ << ",motor" << i << "_cmd_type"
                      << ",motor" << i << "_kp_cmd"
                      << ",motor" << i << "_kd_cmd"
                      << ",motor" << i << "_q_cmd"
                      << ",motor" << i << "_dq_cmd"
                      << ",motor" << i << "_tau_cmd";
    }

    // For each motor, add state columns
    for (size_t i = 0; i < num_motors; ++i) {
        csv_log_file_ << ",motor" << i << "_q_actual"
                      << ",motor" << i << "_dq_actual"
                      << ",motor" << i << "_tau_actual"
                      << ",motor" << i << "_temp_mos"
                      << ",motor" << i << "_temp_rotor"
                      << ",motor" << i << "_error_code";
    }

    csv_log_file_ << std::endl;

    std::cout << "CSV logging enabled: " << log_filename << std::endl;
}

void DMDeviceCollection::disable_csv_logging() {
    if (!csv_logging_enabled_) {
        return;
    }

    if (csv_log_file_.is_open()) {
        csv_log_file_.flush();
        csv_log_file_.close();
    }

    csv_logging_enabled_ = false;
    std::cout << "CSV logging disabled" << std::endl;
}

void DMDeviceCollection::log_command(int motor_index, const std::string& cmd_type,
                                     const MITParam* mit_param,
                                     const PosVelParam* pos_vel_param) {
    if (!csv_logging_enabled_) {
        return;
    }

    CommandLog cmd_log;
    cmd_log.command_type = cmd_type;
    cmd_log.timestamp = std::chrono::steady_clock::now();

    if (mit_param) {
        cmd_log.kp = mit_param->kp;
        cmd_log.kd = mit_param->kd;
        cmd_log.q_cmd = mit_param->q;
        cmd_log.dq_cmd = mit_param->dq;
        cmd_log.tau_cmd = mit_param->tau;
    } else if (pos_vel_param) {
        cmd_log.kp = 0.0;
        cmd_log.kd = 0.0;
        cmd_log.q_cmd = pos_vel_param->position;
        cmd_log.dq_cmd = pos_vel_param->velocity;
        cmd_log.tau_cmd = 0.0;
    }

    last_commands_[motor_index] = cmd_log;
}

void DMDeviceCollection::log_motor_states() {
    if (!csv_logging_enabled_ || !csv_log_file_.is_open()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double relative_time = std::chrono::duration<double>(now - csv_log_start_time_).count();

    csv_log_file_ << std::fixed << std::setprecision(6) << relative_time;

    auto motors = get_motors();

    // Write command data for each motor
    for (size_t i = 0; i < motors.size(); ++i) {
        if (last_commands_.count(i)) {
            const auto& cmd = last_commands_[i];
            csv_log_file_ << "," << cmd.command_type
                          << "," << cmd.kp
                          << "," << cmd.kd
                          << "," << cmd.q_cmd
                          << "," << cmd.dq_cmd
                          << "," << cmd.tau_cmd;
        } else {
            // No command logged yet for this motor
            csv_log_file_ << ",NONE,0,0,0,0,0";
        }
    }

    // Write state data for each motor
    for (const auto& motor : motors) {
        csv_log_file_ << "," << motor.get_position()
                      << "," << motor.get_velocity()
                      << "," << motor.get_torque()
                      << "," << motor.get_state_tmos()
                      << "," << motor.get_state_trotor()
                      << "," << static_cast<int>(motor.get_state_error());
    }

    csv_log_file_ << '\n';

    // Flush every 50 cycles for better performance
    csv_flush_counter_++;
    if (csv_flush_counter_ >= 50) {
        csv_log_file_.flush();
        csv_flush_counter_ = 0;
    }
}

void DMDeviceCollection::init_command_log(int motor_index, const std::string& cmd_type,
                                          double kp, double kd, double q_cmd, double dq_cmd,
                                          double tau_cmd) {
    CommandLog cmd_log;
    cmd_log.command_type = cmd_type;
    cmd_log.kp = kp;
    cmd_log.kd = kd;
    cmd_log.q_cmd = q_cmd;
    cmd_log.dq_cmd = dq_cmd;
    cmd_log.tau_cmd = tau_cmd;
    cmd_log.timestamp = std::chrono::steady_clock::now();
    last_commands_[motor_index] = cmd_log;
}

}  // namespace openarm::damiao_motor
