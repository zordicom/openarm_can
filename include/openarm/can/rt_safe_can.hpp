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

#ifndef OPENARM_CAN__RT_SAFE_CAN_HPP_
#define OPENARM_CAN__RT_SAFE_CAN_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <poll.h>
#include <errno.h>

#include <array>
#include <atomic>
#include <chrono>
#include <vector>

namespace openarm::can {

// RT-safe CAN communication with non-blocking operations and no dynamic allocation.
class RTSafeCANSocket {
public:
    static constexpr size_t MAX_PENDING_FRAMES = 64;

    RTSafeCANSocket() = default;
    ~RTSafeCANSocket();

    // Initialize CAN socket (call from non-RT context).
    bool init(const std::string& interface);

    // Close CAN socket.
    void close();

    // Non-blocking write with timeout (0 = no wait). Returns true if sent.
    bool try_write(const can_frame& frame, int timeout_us = 0);

    // Non-blocking read with timeout (0 = no wait). Returns true if received.
    bool try_read(can_frame& frame, int timeout_us = 0);

    // Batch write multiple frames. Returns number sent.
    size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0);

    // Batch read multiple frames. Returns number received.
    size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0);

    // Check if socket is initialized and ready.
    bool is_ready() const { return socket_fd_ >= 0; }

    // Get socket file descriptor for advanced usage.
    int get_fd() const { return socket_fd_; }

    // Set socket to non-blocking mode.
    bool set_non_blocking(bool enable = true);

    // Get error code from last operation.
    int get_last_error() const { return last_error_; }

private:
    int socket_fd_ = -1;
    int last_error_ = 0;

    // Pre-allocated pollfd for RT-safe polling
    struct pollfd poll_fd_;

    // Wait for socket to be ready for I/O (events: POLLIN for read, POLLOUT for write).
    bool wait_for_io(short events, int timeout_us);
};

// Lock-free single-producer single-consumer ring buffer for CAN frames.
template<typename FrameType, size_t Size>
class RTSafeCANBuffer {
public:
    RTSafeCANBuffer() : head_(0), tail_(0) {}

    // Try to push a frame (non-blocking). Returns false if buffer full.
    bool try_push(const FrameType& frame) {
        size_t next_head = (head_ + 1) % Size;
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Buffer full
        }

        buffer_[head_] = frame;
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    // Try to pop a frame (non-blocking). Returns false if buffer empty.
    bool try_pop(FrameType& frame) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        if (current_tail == head_.load(std::memory_order_acquire)) {
            return false;  // Buffer empty
        }

        frame = buffer_[current_tail];
        tail_.store((current_tail + 1) % Size, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return tail_.load(std::memory_order_acquire) ==
               head_.load(std::memory_order_acquire);
    }

    bool full() const {
        size_t next_head = (head_.load(std::memory_order_acquire) + 1) % Size;
        return next_head == tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (h >= t) ? (h - t) : (Size - t + h);
    }

private:
    std::array<FrameType, Size> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};

} // namespace openarm::can

#endif // OPENARM_CAN__RT_SAFE_CAN_HPP_