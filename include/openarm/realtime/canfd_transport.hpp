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

#ifndef OPENARM_REALTIME_CANFD_TRANSPORT_HPP_
#define OPENARM_REALTIME_CANFD_TRANSPORT_HPP_

#include <linux/can.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <array>
#include <string>

#include "openarm/realtime/transport.hpp"

namespace openarm::realtime {

/**
 * @brief CAN-FD transport implementation
 *
 * Provides CAN-FD support with up to 64 bytes of data per frame.
 * Uses the same batch I/O patterns as regular CAN but with canfd_frame.
 *
 * RAII: Constructor opens CAN-FD socket, destructor closes it.
 */
class CANFDTransport : public IOpenArmTransport {
public:
    static constexpr size_t MAX_PENDING_FRAMES = 64;

    explicit CANFDTransport(const std::string& interface);
    ~CANFDTransport() override;

    // Delete copy/move to prevent socket confusion
    CANFDTransport(const CANFDTransport&) = delete;
    CANFDTransport& operator=(const CANFDTransport&) = delete;
    CANFDTransport(CANFDTransport&&) = delete;
    CANFDTransport& operator=(CANFDTransport&&) = delete;

    size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0) override;
    size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0) override;
    bool is_ready() const override;
    size_t get_max_payload_size() const override;

private:
    int socket_fd_ = -1;

    // Pre-allocated buffers for batch operations (avoid allocation in RT path)
    std::array<struct canfd_frame, MAX_PENDING_FRAMES> send_canfd_frames_;
    std::array<struct canfd_frame, MAX_PENDING_FRAMES> recv_canfd_frames_;
    std::array<struct mmsghdr, MAX_PENDING_FRAMES> send_msgs_;
    std::array<struct iovec, MAX_PENDING_FRAMES> send_iovecs_;
    std::array<struct mmsghdr, MAX_PENDING_FRAMES> recv_msgs_;
    std::array<struct iovec, MAX_PENDING_FRAMES> recv_iovecs_;
};

}  // namespace openarm::realtime

#endif  // OPENARM_REALTIME_CANFD_TRANSPORT_HPP_
