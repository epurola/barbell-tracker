#pragma once 

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace vbt::runtime
{
    template <typename T, std::size_t Capacity>
    class SpscRingBuffer
    {
        static_assert(Capacity >= 2, "Capacity must be ate least 2");
        static_assert((Capacity & (Capacity -1)) == 0, "Capacity must be a power of two");
        static_assert(std::is_trivially_copyable<T>::value, "Queue payload must be trivially copyable");
        static_assert(Capacity < 0x80000000ULL, "Capacity must fit safely in uint32 counters");

        public:
            SpscRingBuffer() = default;

            SpscRingBuffer(const SpscRingBuffer&) = delete;
            SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

            //Call only from single producer task
            [[nodiscard]] bool tryPush(const T& item) noexcept
            {
                const uint32_t head = head_.load(std::memory_order_relaxed);
                const uint32_t tail = tail_.load(std::memory_order_acquire);

                if(static_cast<uint32_t>(head - tail) >= Capacity)
                {
                    dropped_.fetch_add(1, std::memory_order_relaxed);
                    return false;
                }

                storage_[head & kIndexMask] = item;

                head_.store(head + 1, std::memory_order_release);
                return true;
            }

            //Call only from the single consumer task
            [[nodiscard]] bool tryPop(T& item) noexcept
            {
                const uint32_t tail = tail_.load(std::memory_order_relaxed);
                const uint32_t head = head_.load(std::memory_order_acquire);

                if(tail == head) return false;

                item = storage_[tail & kIndexMask];

                tail_.store(tail + 1, std::memory_order_release);
                return true;
            }

            [[nodiscard]] uint32_t droppedCount() const noexcept
            {
                return dropped_.load(std::memory_order_relaxed);
            }

            [[nodiscard]] uint32_t sizeApprox() const noexcept
            {
                const uint32_t head = head_.load(std::memory_order_acquire);
                const uint32_t tail = tail_.load(std::memory_order_acquire);
                return static_cast<uint32_t>(head - tail);
            }

            static constexpr std::size_t capacity() noexcept
            {
                return Capacity;
            }

        private:
            static constexpr uint32_t kIndexMask = static_cast<uint32_t>(Capacity - 1);

            std::array<T, Capacity> storage_{};

            alignas(4) std::atomic<uint32_t> head_{0};
            alignas(4) std::atomic<uint32_t> tail_{0};

            std::atomic<uint32_t> dropped_{0};
    };
}
