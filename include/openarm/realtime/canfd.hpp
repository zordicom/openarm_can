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

#ifndef OPENARM_REALTIME_CANFD_HPP_
#define OPENARM_REALTIME_CANFD_HPP_

#include <linux/can.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <array>
#include <string>

#include "openarm/realtime/transport.hpp"

namespace openarm::realtime::can {

/**
 * @brief CAN-FD socket implementation
 *
 * Provides CAN-FD support with up to 64 bytes of data per frame.
 * Implements IOpenArmTransport interface for CAN-FD.
 *
 * RAII: Constructor opens CAN-FD socket, destructor closes it.
 */
class CANFDSocket : public IOpenArmTransport {
public:
    static constexpr ssize_t MAX_FRAMES = 64;

    explicit CANFDSocket(const std::string& interface);
    ~CANFDSocket() override;

    // Delete copy/move to prevent socket confusion
    CANFDSocket(const CANFDSocket&) = delete;
    CANFDSocket& operator=(const CANFDSocket&) = delete;
    CANFDSocket(CANFDSocket&&) = delete;
    CANFDSocket& operator=(CANFDSocket&&) = delete;

    // Write frames. Returns number sent, -1 if error (check errno)
    ssize_t write_batch(const can_frame* frames, ssize_t count, int timeout_us = 0) override;

    // Read frames. Returns number received, -1 if error (check errno)
    ssize_t read_batch(can_frame* frames, ssize_t max_count, int timeout_us = 0) override;

    size_t get_max_payload_size() const override;

private:
    int socket_fd_ = -1;

    // Pre-allocated buffers for batch operations (avoid allocation in RT path)
    std::array<struct canfd_frame, MAX_FRAMES> send_canfd_frames_;
    std::array<struct canfd_frame, MAX_FRAMES> recv_canfd_frames_;
    std::array<struct mmsghdr, MAX_FRAMES> send_msgs_;
    std::array<struct iovec, MAX_FRAMES> send_iovecs_;
    std::array<struct mmsghdr, MAX_FRAMES> recv_msgs_;
    std::array<struct iovec, MAX_FRAMES> recv_iovecs_;
};

}  // namespace openarm::realtime::can

#endif  // OPENARM_REALTIME_CANFD_HPP_
