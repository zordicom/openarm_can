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

#include "openarm/can/rt_safe_can.hpp"
#include "openarm/damiao_motor/dm_motor.hpp"
#include "openarm/damiao_motor/dm_motor_control.hpp"
#include "openarm/damiao_motor/dm_motor_constants.hpp"

namespace openarm::can {

// RT-safe wrapper for OpenArm motor control with non-blocking operations and pre-allocated buffers.
class RTSafeOpenArm {
public:
    static constexpr size_t MAX_MOTORS = 10;
    static constexpr size_t MAX_CAN_FRAMES = 64;

    RTSafeOpenArm() = default;
    ~RTSafeOpenArm() = default;

    // Initialize the RT-safe OpenArm interface (call from non-RT context).
    bool init(const std::string& can_interface);

    // Close the CAN interface.
    void close();

    // Add a motor to the interface (call from non-RT context). Returns motor index or -1 on failure.
    int add_motor(damiao_motor::MotorType motor_type,
                  uint32_t send_can_id,
                  uint32_t recv_can_id);

    size_t get_motor_count() const { return motor_count_; }

    // RT-safe command methods (non-blocking)

    // Send enable command to all motors. Returns number sent.
    size_t enable_all_motors_rt(int timeout_us = 500);

    // Send disable command to all motors. Returns number sent.
    size_t disable_all_motors_rt(int timeout_us = 500);

    // Send set zero command to all motors. Returns number sent.
    size_t set_zero_all_motors_rt(int timeout_us = 500);

    // Send refresh/state request to all motors. Returns number sent.
    size_t refresh_all_motors_rt(int timeout_us = 500);

    // Write parameter to all motors. Returns number sent.
    size_t write_param_all_rt(openarm::damiao_motor::RID rid, uint32_t value, int timeout_us = 500);

    // Send MIT control commands batch. Returns number sent.
    size_t send_mit_batch_rt(const damiao_motor::MITParam* params,
                             size_t count,
                             int timeout_us = 500);

    // Send position/velocity control commands batch. Returns number sent.
    size_t send_posvel_batch_rt(const damiao_motor::PosVelParam* params,
                                size_t count,
                                int timeout_us = 500);

    // Receive motor states batch. Returns number received.
    size_t receive_states_batch_rt(damiao_motor::StateResult* states,
                                   size_t max_count,
                                   int timeout_us = 500);

    enum class ControlMode {
        MIT,                // MIT control mode
        POSITION_VELOCITY   // Position/Velocity control mode
    };

    // Set control mode for all motors. Returns true if successful for all.
    bool set_mode_all_rt(ControlMode mode, int timeout_us = 500);

    int get_last_error() const { return last_error_; }

    bool is_ready() const { return can_socket_ && can_socket_->is_ready(); }

private:
    // Pre-allocated motor storage (using pointers since Motor has no default constructor)
    std::array<std::unique_ptr<damiao_motor::Motor>, MAX_MOTORS> motors_;
    size_t motor_count_ = 0;

    // Pre-allocated CAN frame buffers
    std::array<can_frame, MAX_CAN_FRAMES> tx_frames_;
    std::array<can_frame, MAX_CAN_FRAMES> rx_frames_;

    // CAN socket
    std::unique_ptr<RTSafeCANSocket> can_socket_;

    // Error tracking
    std::atomic<int> last_error_{0};

    // Motor ID to index mapping for fast lookup
    std::array<int, 256> recv_id_to_motor_index_;  // Assuming CAN IDs < 256

    // Helper methods
    bool encode_mit_command(const damiao_motor::Motor& motor,
                           const damiao_motor::MITParam& param,
                           can_frame& frame);

    bool encode_posvel_command(const damiao_motor::Motor& motor,
                              const damiao_motor::PosVelParam& param,
                              can_frame& frame);

    bool encode_simple_command(const damiao_motor::Motor& motor,
                              uint8_t cmd,
                              can_frame& frame);

    bool decode_motor_state(const damiao_motor::Motor& motor,
                           const can_frame& frame,
                           damiao_motor::StateResult& state);

    // Convert between CAN packet and frame
    void packet_to_frame(const damiao_motor::CANPacket& packet, can_frame& frame);
    void frame_to_data(const can_frame& frame, std::vector<uint8_t>& data);
};

}  // namespace openarm::can

#endif  // OPENARM_CAN__RT_SAFE_OPENARM_HPP_