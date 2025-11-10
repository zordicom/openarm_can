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

#ifndef OPENARM_REALTIME_TRANSPORT_HPP_
#define OPENARM_REALTIME_TRANSPORT_HPP_

#include <linux/can.h>
#include <string>

namespace openarm::realtime {

/**
 * @brief Abstract interface for CAN transport layer
 *
 * This interface allows for different CAN implementations (regular CAN, CAN-FD)
 * while maintaining the same API for the OpenArm layer.
 *
 * RAII: Constructor initializes transport, destructor cleans up.
 */
class IOpenArmTransport {
public:
    virtual ~IOpenArmTransport() = default;

    /**
     * @brief Write frames to the CAN bus
     * @param frames Pointer to array of frames to send
     * @param count Number of frames to send
     * @param timeout_us Timeout in microseconds
     * @return Number of frames successfully sent
     */
    virtual size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0) = 0;

    /**
     * @brief Read frames from the CAN bus
     * @param frames Pointer to array where received frames will be stored
     * @param max_count Maximum number of frames to receive
     * @param timeout_us Timeout in microseconds
     * @return Number of frames successfully received
     */
    virtual size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0) = 0;

    /**
     * @brief Check if transport is ready
     * @return true if ready, false otherwise
     */
    virtual bool is_ready() const = 0;

    /**
     * @brief Get the maximum payload size supported by this transport
     * @return Maximum data bytes per frame (8 for CAN, up to 64 for CAN-FD)
     */
    virtual size_t get_max_payload_size() const = 0;
};

}  // namespace openarm::realtime

#endif  // OPENARM_REALTIME_TRANSPORT_HPP_
