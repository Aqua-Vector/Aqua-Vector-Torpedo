#ifndef STATIC_RING_BUFFER_HPP_
#define STATIC_RING_BUFFER_HPP_

#include "IMessageQueue.hpp"
#include <atomic>
#include <cstddef>

template <typename T, size_t Capacity>
class StaticRingBuffer : public IMessageQueue<T> {
	static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2 for bitwise optimization");

private:
	T buffer_[Capacity];
	std::atomic<size_t> head_{0};
	std::atomic<size_t> tail_{0};

public:
	StaticRingBuffer() = default;
	virtual ~StaticRingBuffer() = default;

	bool push(const T& item) override {
		size_t current_tail = tail_.load(std::memory_order_relaxed);

		size_t next_tail = (current_tail + 1) & (Capacity - 1);

		if (next_tail == head_.load(std::memory_order_acquire)) return false;

		buffer_[current_tail] = item;
		tail_.store(next_tail, std::memory_order_release);
		return true;
	}

	bool pop(T& item) override {
		size_t current_head = head_.load(std::memory_order_relaxed);

		if (current_head == tail_.load(std::memory_order_acquire)) return false;

		item = buffer_[current_head];
		head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
		return true;
	}

	bool is_empty() const override {
		return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
	}
};

#endif /* STATIC_RING_BUFFER_HPP_ */

