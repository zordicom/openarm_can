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

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
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

    explicit CANFDTransport(const std::string& interface) {
        // Create socket with CAN-FD support
        socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Failed to create CAN-FD socket (errno: " +
                                     std::to_string(errno) + ")");
        }

        // Enable CAN-FD mode
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                       sizeof(enable_canfd)) < 0) {
            int err = errno;
            ::close(socket_fd_);
            throw std::runtime_error("Failed to enable CAN-FD mode (errno: " +
                                     std::to_string(err) + ")");
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
            throw std::runtime_error("Failed to bind CAN-FD socket to " + interface +
                                     " (errno: " + std::to_string(err) + ")");
        }

        // Increase socket buffer sizes
        int tx_buf_size = 131072;  // 128KB for larger FD frames
        int rx_buf_size = 131072;  // 128KB

        setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &tx_buf_size, sizeof(tx_buf_size));
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rx_buf_size, sizeof(rx_buf_size));

        // Set non-blocking mode for RT safety
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags < 0) {
            int err = errno;
            ::close(socket_fd_);
            throw std::runtime_error("Failed to get socket flags (errno: " +
                                     std::to_string(err) + ")");
        }

        flags |= O_NONBLOCK;
        if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
            int err = errno;
            ::close(socket_fd_);
            throw std::runtime_error("Failed to set non-blocking mode (errno: " +
                                     std::to_string(err) + ")");
        }
    }

    ~CANFDTransport() override {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    // Delete copy/move to prevent socket confusion
    CANFDTransport(const CANFDTransport&) = delete;
    CANFDTransport& operator=(const CANFDTransport&) = delete;
    CANFDTransport(CANFDTransport&&) = delete;
    CANFDTransport& operator=(CANFDTransport&&) = delete;

    size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0) override {
        if (!frames || count == 0 || socket_fd_ < 0) {
            return 0;
        }

        // Cap to our pre-allocated buffer size
        count = std::min(count, MAX_PENDING_FRAMES);

        auto start_time = std::chrono::steady_clock::now();

        // Convert can_frame to canfd_frame and setup mmsghdr structures
        for (size_t i = 0; i < count; ++i) {
            // Convert standard CAN frame to CAN-FD frame
            memset(&send_canfd_frames_[i], 0, sizeof(struct canfd_frame));
            send_canfd_frames_[i].can_id = frames[i].can_id;
            send_canfd_frames_[i].len = frames[i].len;
            memcpy(send_canfd_frames_[i].data, frames[i].data, frames[i].len);
            // Don't set CANFD_BRS or CANFD_ESI for compatibility with standard CAN

            // Setup iovec to point to the CAN-FD frame
            send_iovecs_[i].iov_base = &send_canfd_frames_[i];
            send_iovecs_[i].iov_len = sizeof(struct canfd_frame);

            // Setup mmsghdr
            memset(&send_msgs_[i], 0, sizeof(struct mmsghdr));
            send_msgs_[i].msg_hdr.msg_iov = &send_iovecs_[i];
            send_msgs_[i].msg_hdr.msg_iovlen = 1;
        }

        size_t sent = 0;

        while (sent < count) {
            // Try to send remaining frames using sendmmsg (batch syscall)
            int ret = sendmmsg(socket_fd_, &send_msgs_[sent], count - sent, MSG_DONTWAIT);

            if (ret > 0) {
                sent += ret;
            } else if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // TX buffer full - wait for space with ppoll if timeout allows
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - start_time)
                                   .count();
                if (elapsed >= timeout_us) {
                    return sent;  // Timeout - return partial results
                }

                // Wait for socket to be writable
                struct pollfd pfd;
                pfd.fd = socket_fd_;
                pfd.events = POLLOUT;
                pfd.revents = 0;

                int64_t remaining_us = timeout_us - elapsed;
                struct timespec timeout;
                timeout.tv_sec = remaining_us / 1000000;
                timeout.tv_nsec = (remaining_us % 1000000) * 1000;

                if (ppoll(&pfd, 1, &timeout, nullptr) <= 0) {
                    return sent;  // Timeout or error
                }
                // Loop and try sendmmsg again
            } else {
                // Error (other than EAGAIN) - return what we've sent so far
                return sent;
            }
        }

        return sent;
    }

    size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0) override {
        if (!frames || max_count == 0 || socket_fd_ < 0) {
            return 0;
        }

        // Cap to our pre-allocated buffer size
        max_count = std::min(max_count, MAX_PENDING_FRAMES);

        auto start_time = std::chrono::steady_clock::now();

        // Setup mmsghdr structures using pre-allocated buffers
        for (size_t i = 0; i < max_count; ++i) {
            recv_iovecs_[i].iov_base = &recv_canfd_frames_[i];
            recv_iovecs_[i].iov_len = sizeof(struct canfd_frame);

            memset(&recv_msgs_[i], 0, sizeof(struct mmsghdr));
            recv_msgs_[i].msg_hdr.msg_iov = &recv_iovecs_[i];
            recv_msgs_[i].msg_hdr.msg_iovlen = 1;
        }

        // Try to receive all available frames (non-blocking)
        int ret = recvmmsg(socket_fd_, recv_msgs_.data(), max_count, MSG_DONTWAIT, nullptr);

        if (ret > 0) {
            // Convert received CAN-FD frames to can_frame
            for (int i = 0; i < ret; ++i) {
                // Check if we received CANFD_MTU or CAN_MTU
                if (recv_msgs_[i].msg_len == CANFD_MTU || recv_msgs_[i].msg_len == CAN_MTU) {
                    frames[i].can_id = recv_canfd_frames_[i].can_id;
                    frames[i].len = std::min(recv_canfd_frames_[i].len, static_cast<__u8>(8));
                    memcpy(frames[i].data, recv_canfd_frames_[i].data, frames[i].len);
                }
            }
            return ret;
        }

        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // No data available - wait if timeout allows
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();
            if (elapsed >= timeout_us) {
                return 0;  // Timeout
            }

            // Wait for socket to be readable
            struct pollfd pfd;
            pfd.fd = socket_fd_;
            pfd.events = POLLIN;
            pfd.revents = 0;

            int64_t remaining_us = timeout_us - elapsed;
            struct timespec timeout;
            timeout.tv_sec = remaining_us / 1000000;
            timeout.tv_nsec = (remaining_us % 1000000) * 1000;

            if (ppoll(&pfd, 1, &timeout, nullptr) <= 0) {
                return 0;  // Timeout or error
            }

            // Try again after poll
            ret = recvmmsg(socket_fd_, recv_msgs_.data(), max_count, MSG_DONTWAIT, nullptr);
            if (ret > 0) {
                for (int i = 0; i < ret; ++i) {
                    if (recv_msgs_[i].msg_len == CANFD_MTU || recv_msgs_[i].msg_len == CAN_MTU) {
                        frames[i].can_id = recv_canfd_frames_[i].can_id;
                        frames[i].len = std::min(recv_canfd_frames_[i].len, static_cast<__u8>(8));
                        memcpy(frames[i].data, recv_canfd_frames_[i].data, frames[i].len);
                    }
                }
                return ret;
            }
        }

        // Error or no data
        return 0;
    }

    bool is_ready() const override { return socket_fd_ >= 0; }

    size_t get_max_payload_size() const override {
        return 64;  // CAN-FD supports up to 64 bytes
    }

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
