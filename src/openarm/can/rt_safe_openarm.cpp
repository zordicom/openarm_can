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

#include <openarm/can/rt_safe_openarm.hpp>

#include <algorithm>
#include <cstring>

namespace openarm::can {

bool RTSafeOpenArm::init(const std::string& can_interface) {
    // Initialize CAN socket
    can_socket_ = std::make_unique<RTSafeCANSocket>();
    if (!can_socket_->init(can_interface)) {
        last_error_ = can_socket_->get_last_error();
        return false;
    }

    // Initialize motor ID lookup table
    std::fill(recv_id_to_motor_index_.begin(), recv_id_to_motor_index_.end(), -1);

    motor_count_ = 0;
    return true;
}

void RTSafeOpenArm::close() {
    if (can_socket_) {
        can_socket_->close();
    }
}

int RTSafeOpenArm::add_motor(damiao_motor::MotorType motor_type,
                             uint32_t send_can_id,
                             uint32_t recv_can_id) {
    if (motor_count_ >= MAX_MOTORS) {
        last_error_ = -1;  // Too many motors
        return -1;
    }

    // Create motor object
    motors_[motor_count_] = std::make_unique<damiao_motor::Motor>(motor_type, send_can_id, recv_can_id);

    // Update lookup table for fast state decoding
    if (recv_can_id < recv_id_to_motor_index_.size()) {
        recv_id_to_motor_index_[recv_can_id] = motor_count_;
    }

    return motor_count_++;
}

size_t RTSafeOpenArm::enable_all_motors_rt(int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready()) {
        return 0;
    }

    size_t sent = 0;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] && encode_simple_command(*motors_[i], 0xFC, tx_frames_[i])) {
            sent++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), sent, timeout_us);
}

size_t RTSafeOpenArm::disable_all_motors_rt(int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready()) {
        return 0;
    }

    size_t sent = 0;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] && encode_simple_command(*motors_[i], 0xFD, tx_frames_[i])) {
            sent++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), sent, timeout_us);
}

size_t RTSafeOpenArm::set_zero_all_motors_rt(int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready()) {
        return 0;
    }

    size_t sent = 0;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] && encode_simple_command(*motors_[i], 0xFE, tx_frames_[i])) {
            sent++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), sent, timeout_us);
}

size_t RTSafeOpenArm::refresh_all_motors_rt(int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready()) {
        return 0;
    }

    size_t sent = 0;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i]) {
            // Refresh command uses special CAN ID 0x7FF
            // Format: [can_id_lo] [can_id_hi] CC 00 00 00 00 00
            can_frame& frame = tx_frames_[sent];
            frame.can_id = 0x7FF;  // Special refresh CAN ID
            frame.can_dlc = 8;

            // Get motor's send CAN ID and split into low and high bytes
            uint32_t motor_can_id = motors_[i]->get_send_can_id();
            uint8_t can_id_lo = motor_can_id & 0xFF;        // Low 8 bits
            uint8_t can_id_hi = (motor_can_id >> 8) & 0xFF;  // High 8 bits

            frame.data[0] = can_id_lo;   // CAN ID low byte
            frame.data[1] = can_id_hi;   // CAN ID high byte
            frame.data[2] = 0xCC;         // Refresh command identifier
            frame.data[3] = 0x00;
            frame.data[4] = 0x00;
            frame.data[5] = 0x00;
            frame.data[6] = 0x00;
            frame.data[7] = 0x00;

            sent++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), sent, timeout_us);
}

size_t RTSafeOpenArm::write_param_all_rt(openarm::damiao_motor::RID rid, uint32_t value, int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready()) {
        return 0;
    }

    size_t sent = 0;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i]) {
            // Parameter write uses special CAN ID 0x7FF
            // Format: [can_id_lo] [can_id_hi] 0x55 [RID] [value bytes 0-3 little-endian]
            can_frame& frame = tx_frames_[sent];
            frame.can_id = 0x7FF;  // Special command CAN ID
            frame.can_dlc = 8;

            // Get motor's send CAN ID and split into low and high bytes
            uint32_t motor_can_id = motors_[i]->get_send_can_id();
            uint8_t can_id_lo = motor_can_id & 0xFF;        // Low 8 bits
            uint8_t can_id_hi = (motor_can_id >> 8) & 0xFF;  // High 8 bits

            frame.data[0] = can_id_lo;                      // CAN ID low byte
            frame.data[1] = can_id_hi;                      // CAN ID high byte
            frame.data[2] = 0x55;                           // Write param command
            frame.data[3] = static_cast<uint8_t>(rid);      // Parameter ID
            frame.data[4] = (value >> 0) & 0xFF;            // Value byte 0 (LSB)
            frame.data[5] = (value >> 8) & 0xFF;            // Value byte 1
            frame.data[6] = (value >> 16) & 0xFF;           // Value byte 2
            frame.data[7] = (value >> 24) & 0xFF;           // Value byte 3 (MSB)

            sent++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), sent, timeout_us);
}

size_t RTSafeOpenArm::send_mit_batch_rt(const damiao_motor::MITParam* params,
                                        size_t count,
                                        int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready() || !params) {
        return 0;
    }

    size_t to_send = std::min(count, motor_count_);
    size_t prepared = 0;

    for (size_t i = 0; i < to_send; ++i) {
        if (motors_[i] && encode_mit_command(*motors_[i], params[i], tx_frames_[prepared])) {
            prepared++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), prepared, timeout_us);
}

size_t RTSafeOpenArm::send_posvel_batch_rt(const damiao_motor::PosVelParam* params,
                                           size_t count,
                                           int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready() || !params) {
        return 0;
    }

    size_t to_send = std::min(count, motor_count_);
    size_t prepared = 0;

    for (size_t i = 0; i < to_send; ++i) {
        if (motors_[i] && encode_posvel_command(*motors_[i], params[i], tx_frames_[prepared])) {
            prepared++;
        }
    }

    // Send batch
    return can_socket_->write_batch(tx_frames_.data(), prepared, timeout_us);
}

size_t RTSafeOpenArm::receive_states_batch_rt(damiao_motor::StateResult* states,
                                              size_t max_count,
                                              int timeout_us) {
    if (!can_socket_ || !can_socket_->is_ready() || !states) {
        return 0;
    }

    // Receive CAN frames
    size_t received = can_socket_->read_batch(rx_frames_.data(),
                                              std::min(max_count, MAX_CAN_FRAMES),
                                              timeout_us);

    size_t valid_states = 0;

    // Decode received frames
    for (size_t i = 0; i < received; ++i) {
        // Find motor index from CAN ID (standard 11-bit IDs)
        uint32_t can_id = rx_frames_[i].can_id & CAN_SFF_MASK;

        if (can_id < recv_id_to_motor_index_.size()) {
            int motor_idx = recv_id_to_motor_index_[can_id];
            if (motor_idx >= 0 && motor_idx < static_cast<int>(motor_count_) && motors_[motor_idx]) {
                // Check if this is a parameter response (not motor state)
                // Parameter responses have format: [can_id_lo] [can_id_hi] [cmd] [rid] [data...]
                // cmd = 0x33 (query response) or 0x55 (write response)
                if (rx_frames_[i].can_dlc >= 3) {
                    uint8_t cmd = rx_frames_[i].data[2];
                    if (cmd == 0x33 || cmd == 0x55) {
                        // This is a parameter response, skip it (don't decode as motor state)
                        continue;
                    }
                }

                // Decode state for this motor
                if (decode_motor_state(*motors_[motor_idx], rx_frames_[i], states[motor_idx])) {
                    valid_states++;
                }
            }
        }
    }

    return valid_states;
}

bool RTSafeOpenArm::set_mode_all_rt(ControlMode mode, int timeout_us) {
    // DEPRECATED: Mode switching is now done via write_param_all_rt
    // with RID::CTRL_MODE parameter
    // This function is kept for compatibility but should not be used

    // Mode values: 1 = MIT, 2 = POSITION_VELOCITY
    uint32_t mode_value = (mode == ControlMode::MIT) ? 1 : 2;

    // Use the proper parameter write method
    size_t written = write_param_all_rt(
        openarm::damiao_motor::RID::CTRL_MODE, mode_value, timeout_us);

    return written == motor_count_;
}

bool RTSafeOpenArm::encode_mit_command(const damiao_motor::Motor& motor,
                                       const damiao_motor::MITParam& param,
                                       can_frame& frame) {
    // Create MIT control packet using the encoder
    auto packet = damiao_motor::CanPacketEncoder::create_mit_control_command(motor, param);

    // Convert to CAN frame
    packet_to_frame(packet, frame);
    return true;
}

bool RTSafeOpenArm::encode_posvel_command(const damiao_motor::Motor& motor,
                                          const damiao_motor::PosVelParam& param,
                                          can_frame& frame) {
    // Create position/velocity control packet using the encoder
    auto packet = damiao_motor::CanPacketEncoder::create_pos_vel_control_command(motor, param);

    // Convert to CAN frame
    packet_to_frame(packet, frame);
    return true;
}

bool RTSafeOpenArm::encode_simple_command(const damiao_motor::Motor& motor,
                                          uint8_t cmd,
                                          can_frame& frame) {
    // Create simple command packet
    damiao_motor::CANPacket packet;
    packet.send_can_id = motor.get_send_can_id();
    packet.data.resize(8, 0xFF);  // Fill with 0xFF as per protocol
    packet.data[7] = cmd;  // Command byte goes at the END (index 7)

    // Convert to CAN frame
    packet_to_frame(packet, frame);
    return true;
}

bool RTSafeOpenArm::decode_motor_state(const damiao_motor::Motor& motor,
                                       const can_frame& frame,
                                       damiao_motor::StateResult& state) {
    // Convert CAN frame to data vector
    std::vector<uint8_t> data;
    frame_to_data(frame, data);

    // Use the decoder to parse the state
    state = damiao_motor::CanPacketDecoder::parse_motor_state_data(motor, data);
    return state.valid;
}

void RTSafeOpenArm::packet_to_frame(const damiao_motor::CANPacket& packet, can_frame& frame) {
    frame.can_id = packet.send_can_id;  // Standard frame (11-bit ID)
    frame.can_dlc = std::min(packet.data.size(), size_t(8));

    // Copy data
    std::memcpy(frame.data, packet.data.data(), frame.can_dlc);
}

void RTSafeOpenArm::frame_to_data(const can_frame& frame, std::vector<uint8_t>& data) {
    data.resize(frame.can_dlc);
    std::memcpy(data.data(), frame.data, frame.can_dlc);
}

}  // namespace openarm::can