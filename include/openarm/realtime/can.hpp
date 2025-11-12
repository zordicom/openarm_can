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

#ifndef OPENARM_REALTIME_CAN_HPP_
#define OPENARM_REALTIME_CAN_HPP_

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <array>
#include <atomic>
#include <chrono>
#include <vector>

#include "openarm/realtime/transport.hpp"

namespace openarm::realtime::can {

// Implements IOpenArmTransport interface for classical CAN (up to 8 bytes per frame).
class CANSocket : public IOpenArmTransport {
public:
    static constexpr size_t MAX_FRAMES = 64;

    explicit CANSocket(const std::string& interface);
    ~CANSocket() override;

    // Delete copy/move to prevent socket confusion.
    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&&) = delete;
    CANSocket& operator=(CANSocket&&) = delete;

    // Write frames. Returns number sent, -1 if error (check errno)
    size_t write_batch(const can_frame* frames, size_t count, int timeout_us = 0) override;

    // Read frames. Returns number received, -1 if error (check errno)
    size_t read_batch(can_frame* frames, size_t max_count, int timeout_us = 0) override;

    // Get max payload size for classical CAN
    size_t get_max_payload_size() const override { return 8; }

private:
    int socket_fd_ = -1;

    // Pre-allocated buffers for batch operations (avoid allocation in RT path)
    std::array<struct mmsghdr, MAX_FRAMES> send_msgs_;
    std::array<struct iovec, MAX_FRAMES> send_iovecs_;
    std::array<struct mmsghdr, MAX_FRAMES> recv_msgs_;
    std::array<struct iovec, MAX_FRAMES> recv_iovecs_;
};

// Lock-free single-producer single-consumer ring buffer for CAN frames.
template <typename FrameType, size_t Size>
class CANBuffer {
public:
    CANBuffer() : head_(0), tail_(0) {}

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
        return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
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

}  // namespace openarm::realtime::can

#endif  // OPENARM_REALTIME_CAN_HPP_
