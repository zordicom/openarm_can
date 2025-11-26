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

#include <algorithm>
#include <cstring>
#include <openarm/realtime/openarm.hpp>

namespace openarm::realtime {

OpenArm::OpenArm(std::unique_ptr<IOpenArmTransport> transport) : transport_(std::move(transport)) {
    if (!transport_) {
        throw std::runtime_error("Invalid transport provided to OpenArm");
    }

    // Initialize motor ID lookup tables
    std::fill(recv_id_to_motor_index_.begin(), recv_id_to_motor_index_.end(), -1);
    std::fill(send_id_to_motor_index_.begin(), send_id_to_motor_index_.end(), -1);

    motor_count_ = 0;
}

int OpenArm::add_motor(uint32_t send_can_id, uint32_t recv_can_id) {
    if (motor_count_ >= MAX_MOTORS) {
        errno = EINVAL;  // Too many motors
        return -1;
    }

    auto motor = std::make_unique<damiao_motor::Motor>(send_can_id, recv_can_id);
    // Create motor object

    if (recv_can_id >= recv_id_to_motor_index_.size()) {
        errno = EINVAL;  // recv_can_id too large
        return -1;
    }

    recv_id_to_motor_index_[recv_can_id] = motor_count_;
    send_id_to_motor_index_[send_can_id] = motor_count_;

    // Pre-allocate commands that don't change.
    encode_simple_command(*motor, 0xFC, tx_enable_[motor_count_]);
    encode_simple_command(*motor, 0xFD, tx_disable_[motor_count_]);
    encode_simple_command(*motor, 0xFE, tx_zero_[motor_count_]);

    can_frame& frame = tx_refresh_[motor_count_];
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x7FF;
    frame.len = 8;
    frame.data[0] = motor->get_send_can_id() & 0xFF;         // CAN ID low byte
    frame.data[1] = (motor->get_send_can_id() >> 8) & 0xFF;  // CAN ID high byte
    frame.data[2] = 0xCC;                                    // Refresh command identifier

    motors_[motor_count_] = std::move(motor);

    return motor_count_++;
}

ssize_t OpenArm::enable_all_motors_rt(int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    // Load limits from motors if not already loaded
    if (!limits_loaded_) {
        if (!load_limits_from_motors()) {
            std::cerr << "ERROR: Failed to load motor limit parameters" << std::endl;
            errno = EIO;
            return 0;
        }
        limits_loaded_ = true;
    }

    return transport_->write_batch(tx_enable_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::disable_all_motors_rt(int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    return transport_->write_batch(tx_disable_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::set_zero_all_motors_rt(int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    return transport_->write_batch(tx_zero_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::refresh_all_motors_rt(int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    // Send batch
    return transport_->write_batch(tx_refresh_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::write_param_all_rt(openarm::damiao_motor::RID rid, uint32_t value,
                                    int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    for (size_t i = 0; i < motor_count_; ++i) {
        can_frame& frame = tx_cmd_[i];
        frame.can_id = 0x7FF;
        frame.len = 8;

        // Get motor's send CAN ID and split into low and high bytes
        uint32_t m_can_id = motors_[i]->get_send_can_id();

        frame.data[0] = m_can_id & 0xFF;            // CAN ID low byte
        frame.data[1] = (m_can_id >> 8) & 0xFF;     // CAN ID high byte
        frame.data[2] = 0x55;                       // Write param command
        frame.data[3] = static_cast<uint8_t>(rid);  // Parameter ID
        frame.data[4] = (value >> 0) & 0xFF;        // Value byte 0 (LSB)
        frame.data[5] = (value >> 8) & 0xFF;        // Value byte 1
        frame.data[6] = (value >> 16) & 0xFF;       // Value byte 2
        frame.data[7] = (value >> 24) & 0xFF;       // Value byte 3 (MSB)
    }

    // Send batch
    return transport_->write_batch(tx_cmd_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::send_mit_batch_rt(const damiao_motor::MITParam* params, ssize_t count,
                                   int timeout_us) {
    if (!transport_ || !params) {
        errno = EINVAL;
        return 0;
    }

    ssize_t n = std::min(count, static_cast<ssize_t>(motor_count_));

    for (ssize_t i = 0; i < n; ++i) {
        auto packet =
            damiao_motor::CanPacketEncoder::create_mit_control_command(*motors_[i], params[i]);
        packet_to_frame(packet, tx_cmd_[i]);
    }

    // Send batch
    return transport_->write_batch(tx_cmd_.data(), n, timeout_us);
}

ssize_t OpenArm::send_posvel_batch_rt(const damiao_motor::PosVelParam* params, ssize_t count,
                                      int timeout_us) {
    if (!transport_ || !params) {
        errno = EINVAL;
        return 0;
    }

    ssize_t n = std::min(count, static_cast<ssize_t>(motor_count_));

    for (ssize_t i = 0; i < n; ++i) {
        auto packet =
            damiao_motor::CanPacketEncoder::create_pos_vel_control_command(*motors_[i], params[i]);
        packet_to_frame(packet, tx_cmd_[i]);
    }

    // Send batch
    return transport_->write_batch(tx_cmd_.data(), n, timeout_us);
}

ssize_t OpenArm::receive_states_batch_rt(damiao_motor::StateResult* states, ssize_t max_count,
                                         int timeout_us) {
    if (!transport_ || !states) {
        errno = EINVAL;
        return 0;
    }

    for (ssize_t i = 0; i < std::min(max_count, static_cast<ssize_t>(motor_count_)); ++i) {
        states[i].valid = false;
    }

    // Receive CAN frames
    ssize_t n = transport_->read_batch(
        rx_frames_.data(), std::min(max_count, static_cast<ssize_t>(MAX_CAN_FRAMES)), timeout_us);

    // Decode received frames
    for (ssize_t i = 0; i < n; ++i) {
        // Find motor index from CAN ID (standard 11-bit IDs)
        uint32_t can_id = rx_frames_[i].can_id & CAN_SFF_MASK;

        if (can_id >= recv_id_to_motor_index_.size()) {
            fprintf(stderr, "receive_states_batch_rt: CAN ID out of bounds\n");
            // out of bounds
            continue;
        }

        int midx = recv_id_to_motor_index_[can_id];
        if (midx < 0 || midx >= static_cast<int>(motor_count_)) {
            fprintf(
                stderr,
                "receive_states_batch_rt:Motor index out of bounds or not configured (midx=%d)\n",
                midx);
            // out of bounds or not configured
            continue;
        }

        const can_frame& rx = rx_frames_[i];

        // Check if this is a parameter response (not motor state)
        // Parameter responses have format: [can_id_lo] [can_id_hi] [cmd] [rid] [data...]
        // cmd = 0x33 (query response) or 0x55 (write response)
        // First two bytes should match the send_can_id
        uint32_t send_id = motors_[midx]->get_send_can_id();
        if (rx.len >= 3 && (rx.data[2] == 0x33 || rx.data[2] == 0x55) &&
            rx.data[0] == (send_id & 0xFF) && rx.data[1] == ((send_id >> 8) & 0xFF)) {
            fprintf(stderr, "receive_states_batch_rt: parameter response (cmd=0x%02X)\n",
                    rx.data[2]);
            // This is a parameter response, skip it (don't decode as motor state)
            continue;
        }

        states[midx] =
            damiao_motor::CanPacketDecoder::parse_motor_state_data(*motors_[midx], rx.data, rx.len);
    }

    return n;
}

bool OpenArm::set_mode_all_rt(ControlMode mode, int timeout_us) {
    // DEPRECATED: Mode switching is now done via write_param_all_rt
    // with RID::CTRL_MODE parameter
    // This function is kept for compatibility but should not be used

    // Use the proper parameter write method
    ssize_t written = write_param_all_rt(openarm::damiao_motor::RID::CTRL_MODE,
                                         static_cast<uint32_t>(mode), timeout_us);

    return written == static_cast<ssize_t>(motor_count_);
}

void OpenArm::encode_simple_command(const damiao_motor::Motor& motor, uint8_t cmd,
                                    can_frame& frame) {
    // Create simple command packet
    damiao_motor::CANPacket packet;
    packet.send_can_id = motor.get_send_can_id();
    packet.data.resize(8, 0xFF);  // Fill with 0xFF as per protocol
    packet.data[7] = cmd;         // Command byte goes at the END (index 7)

    // Convert to CAN frame
    packet_to_frame(packet, frame);
}

bool OpenArm::load_limits_from_motors(int timeout_ms) {
    if (!transport_) {
        return false;
    }

    // Pre-allocated storage for limit values per motor
    std::array<double, MAX_MOTORS> pmax_values;
    std::array<double, MAX_MOTORS> vmax_values;
    std::array<double, MAX_MOTORS> tmax_values;

    pmax_values.fill(-1.0);
    vmax_values.fill(-1.0);
    tmax_values.fill(-1.0);

    // Query PMAX for all motors
    for (size_t i = 0; i < motor_count_; ++i) {
        auto packet = damiao_motor::CanPacketEncoder::create_query_param_command(
            *motors_[i], static_cast<int>(damiao_motor::RID::PMAX));
        packet_to_frame(packet, tx_cmd_[i]);
    }
    transport_->write_batch(tx_cmd_.data(), motor_count_, 500);

    // Query VMAX for all motors
    for (size_t i = 0; i < motor_count_; ++i) {
        auto packet = damiao_motor::CanPacketEncoder::create_query_param_command(
            *motors_[i], static_cast<int>(damiao_motor::RID::VMAX));
        packet_to_frame(packet, tx_cmd_[i]);
    }
    transport_->write_batch(tx_cmd_.data(), motor_count_, 500);

    // Query TMAX for all motors
    for (size_t i = 0; i < motor_count_; ++i) {
        auto packet = damiao_motor::CanPacketEncoder::create_query_param_command(
            *motors_[i], static_cast<int>(damiao_motor::RID::TMAX));
        packet_to_frame(packet, tx_cmd_[i]);
    }
    transport_->write_batch(tx_cmd_.data(), motor_count_, 500);

    // Receive responses with timeout
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 start_time)
               .count() < timeout_ms) {
        ssize_t n = transport_->read_batch(rx_frames_.data(), MAX_CAN_FRAMES, 1000);

        for (ssize_t i = 0; i < n; ++i) {
            const can_frame& frame = rx_frames_[i];

            // Check if this is a parameter response (cmd = 0x33 for query response)
            if (frame.len < 8 || frame.data[2] != 0x33) {
                continue;
            }

            // Parse using raw bytes
            auto result =
                damiao_motor::CanPacketDecoder::parse_motor_param_data(frame.data, frame.len);

            if (!result.valid) continue;

            // Get motor CAN ID from response (first 2 bytes)
            uint32_t motor_send_can_id =
                (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[0];

            int motor_idx = send_id_to_motor_index_[motor_send_can_id];

            if (motor_idx < 0) continue;

            // Store the value based on RID
            if (result.rid == static_cast<int>(damiao_motor::RID::PMAX)) {
                pmax_values[motor_idx] = result.value;
            } else if (result.rid == static_cast<int>(damiao_motor::RID::VMAX)) {
                vmax_values[motor_idx] = result.value;
            } else if (result.rid == static_cast<int>(damiao_motor::RID::TMAX)) {
                tmax_values[motor_idx] = result.value;
            }
        }
    }

    // Validate and set limits for all motors
    for (size_t i = 0; i < motor_count_; ++i) {
        if (pmax_values[i] <= 0 || vmax_values[i] <= 0 || tmax_values[i] <= 0) {
            std::cerr << "ERROR: Invalid limit parameters from motor "
                      << motors_[i]->get_send_can_id() << std::endl;
            return false;
        }

        damiao_motor::LimitParam limit{pmax_values[i], vmax_values[i], tmax_values[i]};
        motors_[i]->set_limit(limit);

        std::cerr << "Motor " << motors_[i]->get_send_can_id() << " limits: pMax=" << pmax_values[i]
                  << " rad, vMax=" << vmax_values[i] << " rad/s, tMax=" << tmax_values[i] << " Nm"
                  << std::endl;
    }

    return true;
}

ssize_t OpenArm::save_params_to_flash_rt(int timeout_us) {
    if (!transport_) {
        errno = EINVAL;
        return 0;
    }

    // Build save commands for all motors
    // Save command format: CAN ID 0x7FF, data = [motor_id_lo, motor_id_hi, 0xAA, 0, 0, 0, 0, 0]
    for (size_t i = 0; i < motor_count_; ++i) {
        can_frame& frame = tx_cmd_[i];
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = 0x7FF;
        frame.len = 8;

        uint32_t send_id = motors_[i]->get_send_can_id();
        frame.data[0] = send_id & 0xFF;         // Motor ID low byte
        frame.data[1] = (send_id >> 8) & 0xFF;  // Motor ID high byte
        frame.data[2] = 0xAA;                   // Save to flash command
        // data[3-7] = 0 (already zeroed by memset)
    }

    return transport_->write_batch(tx_cmd_.data(), motor_count_, timeout_us);
}

ssize_t OpenArm::save_params_to_flash_one_rt(size_t motor_index, int timeout_us) {
    if (!transport_ || motor_index >= motor_count_) {
        errno = EINVAL;
        return 0;
    }

    // Build save command for single motor
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x7FF;
    frame.len = 8;

    uint32_t send_id = motors_[motor_index]->get_send_can_id();
    frame.data[0] = send_id & 0xFF;         // Motor ID low byte
    frame.data[1] = (send_id >> 8) & 0xFF;  // Motor ID high byte
    frame.data[2] = 0xAA;                   // Save to flash command
    // data[3-7] = 0 (already zeroed by memset)

    return transport_->write_batch(&frame, 1, timeout_us);
}

}  // namespace openarm::realtime
