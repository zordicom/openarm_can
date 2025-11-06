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
#include <openarm/can/rt_safe_can.hpp>

namespace openarm::can {

RTSafeCANSocket::~RTSafeCANSocket() { close(); }

bool RTSafeCANSocket::init(const std::string& interface, bool use_canfd) {
    // Close existing socket if open
    if (socket_fd_ >= 0) {
        close();
    }

    use_canfd_ = use_canfd;

    // Create socket
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        last_error_ = errno;
        return false;
    }

    // Enable CAN-FD if requested
    if (use_canfd) {
        int enable = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
            last_error_ = errno;
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
    }

    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        last_error_ = errno;
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
        last_error_ = errno;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set non-blocking mode for RT safety
    set_non_blocking(true);

    // Initialize pollfd structure
    poll_fd_.fd = socket_fd_;
    poll_fd_.events = 0;
    poll_fd_.revents = 0;

    return true;
}

void RTSafeCANSocket::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool RTSafeCANSocket::set_non_blocking(bool enable) {
    if (socket_fd_ < 0) {
        return false;
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) {
        last_error_ = errno;
        return false;
    }

    if (enable) {
        flags |= O_NONBLOCK;
    } else {
        flags &= ~O_NONBLOCK;
    }

    if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
        last_error_ = errno;
        return false;
    }

    return true;
}

bool RTSafeCANSocket::wait_for_io(short events, int timeout_us) {
    if (socket_fd_ < 0) {
        return false;
    }

    poll_fd_.events = events;
    poll_fd_.revents = 0;

    // Convert microseconds to milliseconds for poll
    int timeout_ms = (timeout_us == 0) ? 0 : ((timeout_us + 999) / 1000);

    int ret = poll(&poll_fd_, 1, timeout_ms);
    if (ret < 0) {
        last_error_ = errno;
        return false;
    }

    return ret > 0 && (poll_fd_.revents & events);
}

bool RTSafeCANSocket::try_write(const can_frame& frame, int timeout_us) {
    if (socket_fd_ < 0 || use_canfd_) {
        return false;
    }

    // Check if socket is ready for writing
    if (timeout_us > 0 && !wait_for_io(POLLOUT, timeout_us)) {
        return false;
    }

    ssize_t bytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // Would block, not an error
        }
        last_error_ = errno;
        return false;
    }

    return bytes == sizeof(frame);
}

bool RTSafeCANSocket::try_write(const canfd_frame& frame, int timeout_us) {
    if (socket_fd_ < 0 || !use_canfd_) {
        return false;
    }

    // Check if socket is ready for writing
    if (timeout_us > 0 && !wait_for_io(POLLOUT, timeout_us)) {
        return false;
    }

    ssize_t bytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // Would block, not an error
        }
        last_error_ = errno;
        return false;
    }

    return bytes == sizeof(frame);
}

bool RTSafeCANSocket::try_read(can_frame& frame, int timeout_us) {
    if (socket_fd_ < 0 || use_canfd_) {
        return false;
    }

    // Check if data is available
    if (timeout_us > 0 && !wait_for_io(POLLIN, timeout_us)) {
        return false;
    }

    ssize_t bytes = ::read(socket_fd_, &frame, sizeof(frame));
    if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // No data available, not an error
        }
        last_error_ = errno;
        return false;
    }

    return bytes == sizeof(frame);
}

bool RTSafeCANSocket::try_read(canfd_frame& frame, int timeout_us) {
    if (socket_fd_ < 0 || !use_canfd_) {
        return false;
    }

    // Check if data is available
    if (timeout_us > 0 && !wait_for_io(POLLIN, timeout_us)) {
        return false;
    }

    ssize_t bytes = ::read(socket_fd_, &frame, sizeof(frame));
    if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // No data available, not an error
        }
        last_error_ = errno;
        return false;
    }

    return bytes > 0;  // CAN-FD frames can have variable size
}

size_t RTSafeCANSocket::write_batch(const can_frame* frames, size_t count, int timeout_us) {
    if (!frames || count == 0) {
        return 0;
    }

    size_t sent = 0;
    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < count; ++i) {
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Timeout
            }
        }

        if (try_write(frames[i], remaining_timeout)) {
            ++sent;
        } else {
            break;  // Failed to send
        }
    }

    return sent;
}

size_t RTSafeCANSocket::write_batch(const canfd_frame* frames, size_t count, int timeout_us) {
    if (!frames || count == 0) {
        return 0;
    }

    size_t sent = 0;
    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < count; ++i) {
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Timeout
            }
        }

        if (try_write(frames[i], remaining_timeout)) {
            ++sent;
        } else {
            break;  // Failed to send
        }
    }

    return sent;
}

size_t RTSafeCANSocket::read_batch(can_frame* frames, size_t max_count, int timeout_us) {
    if (!frames || max_count == 0) {
        return 0;
    }

    size_t received = 0;
    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < max_count; ++i) {
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Timeout
            }
        }

        if (try_read(frames[i], remaining_timeout)) {
            ++received;
        } else {
            break;  // No more frames available
        }
    }

    return received;
}

size_t RTSafeCANSocket::read_batch(canfd_frame* frames, size_t max_count, int timeout_us) {
    if (!frames || max_count == 0) {
        return 0;
    }

    size_t received = 0;
    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < max_count; ++i) {
        int remaining_timeout = 0;
        if (timeout_us > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();
            remaining_timeout = timeout_us - elapsed;
            if (remaining_timeout <= 0) {
                break;  // Timeout
            }
        }

        if (try_read(frames[i], remaining_timeout)) {
            ++received;
        } else {
            break;  // No more frames available
        }
    }

    return received;
}

}  // namespace openarm::can
