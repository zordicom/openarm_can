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

/**
 * @brief RT-safe wrapper for OpenArm motor control
 *
 * This class provides deterministic, real-time safe operations for controlling
 * Damiao motors over CAN bus. All operations are non-blocking with pre-allocated
 * buffers to ensure RT safety.
 */
class RTSafeOpenArm {
public:
    static constexpr size_t MAX_MOTORS = 10;
    static constexpr size_t MAX_CAN_FRAMES = 64;

    RTSafeOpenArm() = default;
    ~RTSafeOpenArm() = default;

    /**
     * @brief Initialize the RT-safe OpenArm interface (call from non-RT context)
     * @param can_interface CAN interface name (e.g., "can0")
     * @return true if initialization successful
     */
    bool init(const std::string& can_interface);

    /**
     * @brief Close the CAN interface
     */
    void close();

    /**
     * @brief Add a motor to the interface (call from non-RT context)
     * @param motor_type Type of Damiao motor
     * @param send_can_id CAN ID for sending commands
     * @param recv_can_id CAN ID for receiving states
     * @return Motor index (or -1 on failure)
     */
    int add_motor(damiao_motor::MotorType motor_type,
                  uint32_t send_can_id,
                  uint32_t recv_can_id);

    /**
     * @brief Get number of configured motors
     */
    size_t get_motor_count() const { return motor_count_; }

    // RT-safe command methods (non-blocking)

    /**
     * @brief Send enable command to all motors (RT-safe)
     * @param timeout_us Timeout in microseconds
     * @return Number of commands successfully sent
     */
    size_t enable_all_motors_rt(int timeout_us = 500);

    /**
     * @brief Send disable command to all motors (RT-safe)
     * @param timeout_us Timeout in microseconds
     * @return Number of commands successfully sent
     */
    size_t disable_all_motors_rt(int timeout_us = 500);

    /**
     * @brief Send set zero command to all motors (RT-safe)
     * @param timeout_us Timeout in microseconds
     * @return Number of commands successfully sent
     */
    size_t set_zero_all_motors_rt(int timeout_us = 500);

    /**
     * @brief Send refresh/state request to all motors (RT-safe)
     * @param timeout_us Timeout in microseconds
     * @return Number of refresh commands successfully sent
     */
    size_t refresh_all_motors_rt(int timeout_us = 500);

    /**
     * @brief Send MIT control commands to all motors (RT-safe batch)
     * @param params Array of MIT parameters for each motor
     * @param count Number of motors to command
     * @param timeout_us Timeout in microseconds
     * @return Number of commands successfully sent
     */
    size_t send_mit_batch_rt(const damiao_motor::MITParam* params,
                             size_t count,
                             int timeout_us = 500);

    /**
     * @brief Send position/velocity control commands to all motors (RT-safe batch)
     * @param params Array of position/velocity parameters for each motor
     * @param count Number of motors to command
     * @param timeout_us Timeout in microseconds
     * @return Number of commands successfully sent
     */
    size_t send_posvel_batch_rt(const damiao_motor::PosVelParam* params,
                                size_t count,
                                int timeout_us = 500);

    /**
     * @brief Receive motor states (RT-safe batch)
     * @param states Array to receive motor states
     * @param max_count Maximum number of states to receive
     * @param timeout_us Timeout in microseconds
     * @return Number of states successfully received
     */
    size_t receive_states_batch_rt(damiao_motor::StateResult* states,
                                   size_t max_count,
                                   int timeout_us = 500);

    /**
     * @brief Control modes supported by the motors
     */
    enum class ControlMode {
        MIT,                // MIT control mode
        POSITION_VELOCITY   // Position/Velocity control mode
    };

    /**
     * @brief Set control mode for all motors (RT-safe)
     * @param mode Control mode (MIT or POSITION_VELOCITY)
     * @param timeout_us Timeout in microseconds
     * @return true if mode change successful for all motors
     */
    bool set_mode_all_rt(ControlMode mode, int timeout_us = 500);

    /**
     * @brief Get last error code
     */
    int get_last_error() const { return last_error_; }

    /**
     * @brief Check if interface is ready
     */
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