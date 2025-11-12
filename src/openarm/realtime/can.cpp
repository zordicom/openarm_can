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

#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <openarm/realtime/can.hpp>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;

namespace openarm::realtime::can {

CANSocket::CANSocket(const std::string& interface) {
    // Create socket
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create CAN socket (errno: " + std::to_string(errno) +
                                 ")");
    }

    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        ::close(socket_fd_);
        throw std::runtime_error("Failed to get interface index for " + interface +
                                 " (errno: " + std::to_string(err) + ")");
    }

    // Bind socket to interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        int err = errno;
        ::close(socket_fd_);
        throw std::runtime_error("Failed to bind CAN socket to " + interface +
                                 " (errno: " + std::to_string(err) + ")");
    }

    // Increase socket buffer sizes to prevent drops during burst I/O
    // TX buffer: Need room for at least 8 frames (8 motors) + margin
    // RX buffer: Need room for responses that arrive while we're sending
    static constexpr int tx_buf_size = 64 * 1024;  // 64KB
    static constexpr int rx_buf_size = 64 * 1024;  // 64KB

    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &tx_buf_size, sizeof(tx_buf_size));
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rx_buf_size, sizeof(rx_buf_size));

    // Set non-blocking mode for RT safety
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) {
        int err = errno;
        ::close(socket_fd_);
        throw std::runtime_error("Failed to get socket flags (errno: " + std::to_string(err) + ")");
    }

    flags |= O_NONBLOCK;

    if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
        int err = errno;
        ::close(socket_fd_);
        throw std::runtime_error("Failed to set non-blocking mode (errno: " + std::to_string(err) +
                                 ")");
    }
}

CANSocket::~CANSocket() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

size_t CANSocket::write_batch(const can_frame* frames, size_t count, int timeout_us) {
    if (!frames || count == 0 || socket_fd_ < 0) {
        return 0;
    }

    // Cap to our pre-allocated buffer size
    count = std::min(count, MAX_FRAMES);

    auto start_time = steady_clock::now();

    // Setup mmsghdr structures using pre-allocated buffers
    for (size_t i = 0; i < count; ++i) {
        send_iovecs_[i].iov_base = const_cast<can_frame*>(&frames[i]);
        send_iovecs_[i].iov_len = sizeof(can_frame);

        memset(&send_msgs_[i], 0, sizeof(struct mmsghdr));
        send_msgs_[i].msg_hdr.msg_iov = &send_iovecs_[i];
        send_msgs_[i].msg_hdr.msg_iovlen = 1;
    }

    size_t sent = 0;

    while (sent < count) {
        // Try to send remaining frames
        int ret = sendmmsg(socket_fd_, &send_msgs_[sent], count - sent, MSG_DONTWAIT);

        if (ret > 0) {
            sent += ret;
        } else if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // TX buffer full - wait for space with ppoll if timeout allows
            auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
            if (elapsed >= timeout_us) {
                return sent;  // Timeout - return partial results
            }

            // Wait for socket to be writable
            struct pollfd pfd;
            pfd.fd = socket_fd_;
            pfd.events = POLLOUT;
            pfd.revents = 0;

            struct timespec timeout;
            int64_t remaining_us = timeout_us - elapsed;
            timeout.tv_sec = remaining_us / 1000000;
            timeout.tv_nsec = (remaining_us % 1000000) * 1000;

            if (ppoll(&pfd, 1, &timeout, nullptr) <= 0) {
                return sent;  // Timeout - return partial results
            }
            // Loop and try sendmmsg again
        } else {
            // Error (other than EAGAIN) - errno is set, return -1
            return -1;
        }
    }

    return sent;
}

size_t CANSocket::read_batch(can_frame* frames, size_t max_count, int timeout_us) {
    if (!frames || max_count == 0 || socket_fd_ < 0) {
        return 0;
    }

    // Cap to our pre-allocated buffer size
    max_count = std::min(max_count, MAX_FRAMES);

    auto start_time = steady_clock::now();

    // Setup mmsghdr structures using pre-allocated buffers
    for (size_t i = 0; i < max_count; ++i) {
        recv_iovecs_[i].iov_base = &frames[i];
        recv_iovecs_[i].iov_len = sizeof(can_frame);

        memset(&recv_msgs_[i], 0, sizeof(struct mmsghdr));
        recv_msgs_[i].msg_hdr.msg_iov = &recv_iovecs_[i];
        recv_msgs_[i].msg_hdr.msg_iovlen = 1;
    }

    // Try to receive all available frames (non-blocking)
    int ret = recvmmsg(socket_fd_, recv_msgs_.data(), max_count, MSG_DONTWAIT, nullptr);

    if (ret > 0) {
        return ret;  // Got frames immediately
    }

    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // No data available - wait if timeout allows
        auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
        if (elapsed >= timeout_us) {
            return 0;  // Timeout - no data available
        }

        // Wait for socket to be readable
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;

        struct timespec timeout;
        int64_t remaining_us = timeout_us - elapsed;
        timeout.tv_sec = remaining_us / 1000000;
        timeout.tv_nsec = (remaining_us % 1000000) * 1000;

        if (ppoll(&pfd, 1, &timeout, nullptr) <= 0) {
            return 0;  // Timeout - no data available
        }

        // Try again after poll
        ret = recvmmsg(socket_fd_, recv_msgs_.data(), max_count, MSG_DONTWAIT, nullptr);
        if (ret > 0) {
            return ret;
        }
        // After waiting, still no data or error
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return 0;  // Timeout - no data available
        }
    }

    // Error - errno is set, return -1
    return -1;
}

}  // namespace openarm::realtime::can
