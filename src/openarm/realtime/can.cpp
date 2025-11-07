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

bool CANSocket::try_write(const can_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }

    // Single write attempt - fail fast if TX buffer is full (RT-safe, no spinning)
    ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
    if (n == sizeof(frame)) {
        return true;  // Success
    }

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // TX buffer full - fail immediately instead of spinning
            return false;
        } else {
            last_errno_ = errno;
            return false;
        }
    } else {
        // Partial write, shouldn't happen. Indicates error (see socketcan kernel documentation)
        return false;
    }
}

bool CANSocket::try_read(can_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }

    // Single read attempt - fail fast if no data available (RT-safe, no spinning)
    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));

    if (n == sizeof(frame)) {
        return true;
    }

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available - fail immediately instead of spinning
            return false;
        } else {
            last_errno_ = errno;
            return false;
        }
    } else if (n < static_cast<ssize_t>(sizeof(frame))) {
        // Partial read, shouldn't happen. Indicates error (see socketcan kernel documentation)
        return false;
    }

    return false;
}

size_t CANSocket::write_batch(const can_frame* frames, size_t count, int timeout_us) {
    if (!frames || count == 0) {
        return 0;
    }

    size_t sent = 0;
    auto start_time = steady_clock::now();

    for (size_t i = 0; i < count; ++i) {
        // Check global timeout if specified
        if (timeout_us > 0) {
            auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
            if (elapsed >= timeout_us) {
                break;  // Timeout
            }
        }

        if (try_write(frames[i])) {
            ++sent;
        } else {
            break;  // Failed to send (TX buffer full)
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
        // Check global timeout if specified
        if (timeout_us > 0) {
            auto elapsed = duration_cast<microseconds>(steady_clock::now() - start_time).count();
            if (elapsed >= timeout_us) {
                break;  // Timeout
            }
        }

        if (try_read(frames[i])) {
            ++received;
        } else {
            break;  // No more frames available (RX buffer empty)
        }
    }

    return received;
}

}  // namespace openarm::realtime::can
