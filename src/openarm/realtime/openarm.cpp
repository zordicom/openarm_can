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

OpenArm::OpenArm(std::unique_ptr<IOpenArmTransport> transport)
    : transport_(std::move(transport)) {
    if (!transport_ || !transport_->is_ready()) {
        throw std::runtime_error("Invalid or unready transport provided to OpenArm");
    }

    // Initialize motor ID lookup table
    std::fill(recv_id_to_motor_index_.begin(), recv_id_to_motor_index_.end(), -1);

    motor_count_ = 0;
}

int OpenArm::add_motor(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                       uint32_t recv_can_id) {
    if (motor_count_ >= MAX_MOTORS) {
        last_errno_ = -1;  // Too many motors
        return -1;
    }

    auto motor = std::make_unique<damiao_motor::Motor>(motor_type, send_can_id, recv_can_id);
    // Create motor object

    if (recv_can_id >= recv_id_to_motor_index_.size()) {
        last_errno_ = -1;  // recv_can_id too large.
        return -1;
    }

    recv_id_to_motor_index_[recv_can_id] = motor_count_;

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

size_t OpenArm::enable_all_motors_rt(int timeout_us) {
    if (!transport_ || !transport_->is_ready()) {
        return 0;
    }

    return transport_->write_batch(tx_enable_.data(), motor_count_, timeout_us);
}

size_t OpenArm::disable_all_motors_rt(int timeout_us) {
    if (!transport_ || !transport_->is_ready()) {
        return 0;
    }

    return transport_->write_batch(tx_disable_.data(), motor_count_, timeout_us);
}

size_t OpenArm::set_zero_all_motors_rt(int timeout_us) {
    if (!transport_ || !transport_->is_ready()) {
        return 0;
    }

    return transport_->write_batch(tx_zero_.data(), motor_count_, timeout_us);
}

size_t OpenArm::refresh_all_motors_rt(int timeout_us) {
    if (!transport_ || !transport_->is_ready()) {
        return 0;
    }

    // Send batch
    return transport_->write_batch(tx_refresh_.data(), motor_count_, timeout_us);
}

size_t OpenArm::write_param_all_rt(openarm::damiao_motor::RID rid, uint32_t value, int timeout_us) {
    if (!transport_ || !transport_->is_ready()) {
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

size_t OpenArm::send_mit_batch_rt(const damiao_motor::MITParam* params, size_t count,
                                  int timeout_us) {
    if (!transport_ || !transport_->is_ready() || !params) {
        return 0;
    }

    size_t n = std::min(count, motor_count_);

    for (size_t i = 0; i < n; ++i) {
        auto packet =
            damiao_motor::CanPacketEncoder::create_mit_control_command(*motors_[i], params[i]);
        packet_to_frame(packet, tx_cmd_[i]);
    }

    // Send batch
    return transport_->write_batch(tx_cmd_.data(), n, timeout_us);
}

size_t OpenArm::send_posvel_batch_rt(const damiao_motor::PosVelParam* params, size_t count,
                                     int timeout_us) {
    if (!transport_ || !transport_->is_ready() || !params) {
        return 0;
    }

    size_t n = std::min(count, motor_count_);

    for (size_t i = 0; i < n; ++i) {
        auto packet =
            damiao_motor::CanPacketEncoder::create_pos_vel_control_command(*motors_[i], params[i]);
        packet_to_frame(packet, tx_cmd_[i]);
    }

    // Send batch
    return transport_->write_batch(tx_cmd_.data(), n, timeout_us);
}

size_t OpenArm::receive_states_batch_rt(damiao_motor::StateResult* states, size_t max_count,
                                        int timeout_us) {
    if (!transport_ || !transport_->is_ready() || !states) {
        return 0;
    }

    for (size_t i = 0; i < std::min(max_count, motor_count_); ++i) {
        states[i].valid = false;
    }

    // Receive CAN frames
    size_t n =
        transport_->read_batch(rx_frames_.data(), std::min(max_count, MAX_CAN_FRAMES), timeout_us);

    // Decode received frames
    for (size_t i = 0; i < n; ++i) {
        // Find motor index from CAN ID (standard 11-bit IDs)
        uint32_t can_id = rx_frames_[i].can_id & CAN_SFF_MASK;
        if (can_id >= recv_id_to_motor_index_.size()) {
            // out of bounds
            continue;
        }

        int midx = recv_id_to_motor_index_[can_id];
        if (midx < 0 || midx >= static_cast<int>(motor_count_)) {
            // out of bounds or not configured
            continue;
        }

        const can_frame& rx = rx_frames_[i];

        // Check if this is a parameter response (not motor state)
        // Parameter responses have format: [can_id_lo] [can_id_hi] [cmd] [rid] [data...]
        // cmd = 0x33 (query response) or 0x55 (write response)
        if (rx.len >= 3 && (rx.data[2] == 0x33 || rx.data[2] == 0x55)) {
            // This is a parameter response, skip it (don't decode as motor state)
            continue;
        }

        states[midx] = damiao_motor::CanPacketDecoder::parse_motor_state_data(
            *motors_[midx], rx.data, rx.len);
    }

    return n;
}

bool OpenArm::set_mode_all_rt(ControlMode mode, int timeout_us) {
    // DEPRECATED: Mode switching is now done via write_param_all_rt
    // with RID::CTRL_MODE parameter
    // This function is kept for compatibility but should not be used

    // Use the proper parameter write method
    size_t written = write_param_all_rt(openarm::damiao_motor::RID::CTRL_MODE,
                                        static_cast<uint32_t>(mode), timeout_us);

    return written == motor_count_;
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

}  // namespace openarm::realtime
