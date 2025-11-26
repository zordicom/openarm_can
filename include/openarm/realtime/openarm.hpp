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

#ifndef OPENARM_CAN__RT_SAFE_OPENARM_HPP_
#define OPENARM_CAN__RT_SAFE_OPENARM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>

#include "openarm/damiao_motor/dm_motor.hpp"
#include "openarm/damiao_motor/dm_motor_constants.hpp"
#include "openarm/damiao_motor/dm_motor_control.hpp"
#include "openarm/realtime/transport.hpp"

namespace openarm::realtime {

enum class ControlMode : uint32_t {
    MIT = 1,               // MIT control mode
    POSITION_VELOCITY = 2  // Position/Velocity control mode
};

// RT-safe wrapper for OpenArm motor control with non-blocking operations and pre-allocated buffers.
class OpenArm {
public:
    static constexpr size_t MAX_MOTORS = 10;
    static constexpr size_t MAX_CAN_FRAMES = 64;
    static constexpr size_t MAX_MOTOR_CAN_ID = 256;

    OpenArm() = default;
    explicit OpenArm(std::unique_ptr<IOpenArmTransport> transport);
    ~OpenArm() = default;

    // Add a motor to the interface (call from non-RT context). Returns motor index or -1 on
    // failure.
    int add_motor(uint32_t send_can_id, uint32_t recv_can_id);

    size_t get_motor_count() const { return motor_count_; }

    // RT-safe command methods (non-blocking)
    // Send enable command to all motors. Returns number sent.
    ssize_t enable_all_motors_rt(int timeout_us = 500);

    // Send disable command to all motors. Returns number sent.
    ssize_t disable_all_motors_rt(int timeout_us = 500);

    // Send set zero command to all motors. Returns number sent.
    ssize_t set_zero_all_motors_rt(int timeout_us = 500);

    // Send refresh/state request to all motors. Returns number sent.
    ssize_t refresh_all_motors_rt(int timeout_us = 500);

    // Write parameter to all motors. Returns number sent.
    ssize_t write_param_all_rt(openarm::damiao_motor::RID rid, uint32_t value,
                               int timeout_us = 500);

    // Send MIT control commands batch. Returns number sent.
    ssize_t send_mit_batch_rt(const damiao_motor::MITParam* params, ssize_t count,
                              int timeout_us = 500);

    // Send position/velocity control commands batch. Returns number sent.
    ssize_t send_posvel_batch_rt(const damiao_motor::PosVelParam* params, ssize_t count,
                                 int timeout_us = 500);

    // Receive motor states batch. Returns number received.
    ssize_t receive_states_batch_rt(damiao_motor::StateResult* states, ssize_t max_count,
                                    int timeout_us = 500);

    // Set control mode for all motors. Returns true if successful for all.
    bool set_mode_all_rt(ControlMode mode, int timeout_us = 500);

    // Save parameters to flash for all motors. Motors must be disabled first.
    // Returns number of save commands sent.
    ssize_t save_params_to_flash_rt(int timeout_us = 500);

    // Save parameters to flash for a single motor by index. Motor must be disabled first.
    // Returns 1 on success, 0 or -1 on failure.
    ssize_t save_params_to_flash_one_rt(size_t motor_index, int timeout_us = 500);

private:
    // Pre-allocated motor storage (using pointers since Motor has no default constructor)
    std::array<std::unique_ptr<damiao_motor::Motor>, MAX_MOTORS> motors_;
    size_t motor_count_ = 0;

    // Track if limits have been loaded
    bool limits_loaded_ = false;

    // Pre-allocated CAN frame buffers
    std::array<can_frame, MAX_MOTORS> tx_cmd_;
    std::array<can_frame, MAX_CAN_FRAMES> rx_frames_;

    std::array<can_frame, MAX_MOTORS> tx_refresh_;
    std::array<can_frame, MAX_MOTORS> tx_enable_;
    std::array<can_frame, MAX_MOTORS> tx_disable_;
    std::array<can_frame, MAX_MOTORS> tx_zero_;

    // Transport layer (CAN or CAN-FD)
    std::unique_ptr<IOpenArmTransport> transport_;

    // Motor CAN ID to motor index. Assumes CAN IDs don't exceed MAX_MOTOR_CAN_ID.
    std::array<int, MAX_MOTOR_CAN_ID> recv_id_to_motor_index_;
    std::array<int, MAX_MOTOR_CAN_ID> send_id_to_motor_index_;

    // Helper methods
    void encode_simple_command(const damiao_motor::Motor& motor, uint8_t cmd, can_frame& frame);

    // Read and set limit parameters from all motors (non-RT, called before first enable)
    bool load_limits_from_motors(int timeout_ms = 1000);

    // Convert CAN packet to frame (inline for performance in RT loop)
    inline static void packet_to_frame(const damiao_motor::CANPacket& packet, can_frame& frame) {
        frame.can_id = packet.send_can_id;  // Standard frame (11-bit ID)
        frame.len = std::min(packet.data.size(), size_t(8));
        std::memcpy(frame.data, packet.data.data(), frame.len);
    };
};

}  // namespace openarm::realtime

#endif  // OPENARM_CAN__RT_SAFE_OPENARM_HPP_
