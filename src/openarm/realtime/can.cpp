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

CANSocket::~CANSocket() { close(); }

bool CANSocket::init(const std::string& interface) {
    // Close existing socket if open
    if (socket_fd_ >= 0) {
        close();
    }

    // Create socket
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        last_errno_ = errno;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        last_errno_ = errno;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Bind socket to interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        last_errno_ = errno;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Increase socket buffer sizes to prevent drops during burst I/O
    // TX buffer: Need room for at least 8 frames (8 motors) + margin
    // RX buffer: Need room for responses that arrive while we're sending
    int tx_buf_size = 65536;  // 64KB
    int rx_buf_size = 65536;  // 64KB

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &tx_buf_size, sizeof(tx_buf_size)) < 0) {
        last_errno_ = errno;
        // Non-fatal, continue
    }

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rx_buf_size, sizeof(rx_buf_size)) < 0) {
        last_errno_ = errno;
        // Non-fatal, continue
    }

    // Set non-blocking mode for RT safety
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) {
        last_errno_ = errno;
        return false;
    }

    flags |= O_NONBLOCK;

    if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
        last_errno_ = errno;
        return false;
    }

    return true;
}

void CANSocket::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CANSocket::try_write(const can_frame& frame, int timeout_us) {
    if (socket_fd_ < 0) {
        return false;
    }

    auto start = steady_clock::now();

    while (true) {
        // Try to write immediately
        ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
        if (n == sizeof(frame)) {
            return true;  // Success
        }

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // TX buffer full - wait for space with ppoll
                auto elapsed = duration_cast<microseconds>(steady_clock::now() - start).count();
                if (elapsed >= timeout_us) {
                    return false;  // Timeout
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

                int ret = ppoll(&pfd, 1, &timeout, nullptr);
                if (ret <= 0) {
                    // Timeout or error
                    return false;
                }
                // Loop and try write again
            } else {
                last_errno_ = errno;
                return false;
            }
        } else {
            // Partial write, shouldn't happen. Indicates error (see socketcan kernel documentation)
            return false;
        }
    }
}

bool CANSocket::try_read(can_frame& frame, int timeout_us) {
    if (socket_fd_ < 0) {
        return false;
    }

    auto start = steady_clock::now();

    while (true) {
        // Try to read immediately
        ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));

        if (n == sizeof(frame)) {
            return true;
        }

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available - wait for data with ppoll
                auto elapsed = duration_cast<microseconds>(steady_clock::now() - start).count();
                if (elapsed >= timeout_us) {
                    return false;  // Timeout
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

                int ret = ppoll(&pfd, 1, &timeout, nullptr);
                if (ret <= 0) {
                    // Timeout or error
                    return false;
                }
                // Loop and try read again
            } else {
                last_errno_ = errno;
                return false;
            }
        } else if (n < static_cast<ssize_t>(sizeof(frame))) {
            // Partial read, shouldn't happen. Indicates error (see socketcan kernel documentation)
            return false;
        }
    }
}

size_t CANSocket::write_batch(const can_frame* frames, size_t count, int timeout_us) {
    if (!frames || count == 0) {
        return 0;
    }

    size_t sent = 0;
    auto start_time = steady_clock::now();

    for (size_t i = 0; i < count; ++i) {
        // Calculate remaining timeout for this frame
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Global timeout exceeded
            }
        }

        if (try_write(frames[i], remaining_timeout)) {
            ++sent;
        } else {
            break;  // Failed to send within timeout
        }
    }

    return sent;
}

size_t CANSocket::read_batch(can_frame* frames, size_t max_count, int timeout_us) {
    if (!frames || max_count == 0) {
        return 0;
    }

    size_t received = 0;
    auto start_time = steady_clock::now();

    for (size_t i = 0; i < max_count; ++i) {
        // Calculate remaining timeout for this frame
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Global timeout exceeded
            }
        }

        if (try_read(frames[i], remaining_timeout)) {
            ++received;
        } else {
            break;  // No more frames available within timeout
        }
    }

    return received;
}

}  // namespace openarm::realtime::can
