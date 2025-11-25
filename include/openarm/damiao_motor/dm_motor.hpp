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

#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <optional>

#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
class Motor {
    friend class DMCANDevice;  // Allow MotorDeviceCan to access protected
                               // members
    friend class DMControl;

public:
    // Constructor
    Motor(uint32_t send_can_id, uint32_t recv_can_id);

    // State getters
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }
    uint8_t get_state_error() const { return state_error_; }

    // Error checking
    bool has_unrecoverable_error() const {
        // Error is indicated by upper 3 bits of status nibble (any non-zero value in bits 1-3)
        return state_error_ != 0;
    }

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }

    // Enable status getters
    bool is_enabled() const { return enabled_; }

    // Parameter methods
    double get_param(int RID) const;

    // Limit parameter accessors
    std::optional<LimitParam> get_limit() const { return limit_; }
    void set_limit(const LimitParam& limit) { limit_ = limit; }

protected:
    // State update methods
    void update_state(double q, double dq, double tau, int tmos, int trotor, uint8_t error_code);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_temp_param(int RID, int val);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;

    // Motor limits (populated on first enable or manually)
    std::optional<LimitParam> limit_;

    // Enable status
    bool enabled_;

    // Current state
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;
    uint8_t state_error_;  // Motor error code (0x1 = no error)

    // Parameter storage
    std::map<int, double> temp_param_dict_;
};
}  // namespace openarm::damiao_motor
