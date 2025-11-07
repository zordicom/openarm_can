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

#include <cstring>

#include "openarm/damiao_motor/dm_motor.hpp"
#include "openarm/damiao_motor/dm_motor_control.hpp"

namespace openarm::damiao_motor {

// Implementation of Position-Velocity control command creation
CANPacket CanPacketEncoder::create_pos_vel_control_command(const Motor& motor,
                                                           const PosVelParam& pos_vel_param) {
    CANPacket packet;
    // Position-Velocity mode uses frame ID = 0x100 + motor CAN ID
    packet.send_can_id = 0x100 + motor.get_send_can_id();
    packet.data = pack_pos_vel_control_data(pos_vel_param);
    return packet;
}

std::vector<uint8_t> CanPacketEncoder::pack_pos_vel_control_data(const PosVelParam& pos_vel_param) {
    std::vector<uint8_t> data(8);

    // Convert to float for transmission (motor expects float, not double)
    float p_des = static_cast<float>(pos_vel_param.q);
    float v_des = static_cast<float>(pos_vel_param.dq);

    // Pack floats as bytes (IEEE 754 format, little-endian)
    memcpy(&data[0], &p_des, sizeof(float));
    memcpy(&data[4], &v_des, sizeof(float));

    return data;
}

}  // namespace openarm::damiao_motor
