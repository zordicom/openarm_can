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

#ifndef OPENARM_REALTIME_CAN_TRANSPORT_HPP_
#define OPENARM_REALTIME_CAN_TRANSPORT_HPP_

#include "openarm/realtime/can.hpp"
#include "openarm/realtime/transport.hpp"

namespace openarm::realtime {

/**
 * @brief Regular CAN (Classical CAN) transport implementation
 *
 * Wraps the existing CANSocket to provide IOpenArmTransport interface.
 * Supports standard CAN frames with up to 8 bytes of data.
 *
 * RAII: Constructor opens CAN socket, destructor closes it.
 */
class CANTransport : public IOpenArmTransport {
public:
    explicit CANTransport(const std::string& interface)
        : socket_(interface) {}

    ~CANTransport() override = default;

    // Delete copy/move to prevent socket confusion
    CANTransport(const CANTransport&) = delete;
    CANTransport& operator=(const CANTransport&) = delete;
    CANTransport(CANTransport&&) = delete;
    CANTransport& operator=(CANTransport&&) = delete;

    size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0) override {
        return socket_.write_batch(frames, count, timeout_us);
    }

    size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0) override {
        return socket_.read_batch(frames, max_count, timeout_us);
    }

    bool is_ready() const override {
        return socket_.is_ready();
    }

    size_t get_max_payload_size() const override {
        return 8;  // Classical CAN supports up to 8 bytes
    }

private:
    can::CANSocket socket_;
};

}  // namespace openarm::realtime

#endif  // OPENARM_REALTIME_CAN_TRANSPORT_HPP_
