#ifndef THREAD_SAFE_QUEUE_HPP_
#define THREAD_SAFE_QUEUE_HPP_

#include "IMessageQueue.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue : public IMessageQueue<T> {
private:
	std::queue<T> queue_;
	mutable std::mutex mtx_;
	std::condition_variable cv_;
	bool is_aborted_;

public:
	ThreadSafeQueue() : is_aborted_(false) {}
	~ThreadSafeQueue() override { abort(); }

	bool push(const T& item) override {
		{
			std::lock_guard<std::mutex> lock(mtx_);
			if (is_aborted_) return false;
			queue_.push(item);
		}
		cv_.notify_one();
		return true;
	}

	bool pop(T& item) override {
		std::unique_lock<std::mutex> lock(mtx_);

		cv_.wait(lock, [this]() { return !queue_.empty() || is_aborted_; });

		if (is_aborted_ && queue_.empty()) {
			return false;
		}

		item = queue_.front();
		queue_.pop();
		return true;
	}

	bool is_empty() const override {
		std::lock_guard<std::mutex> lock(mtx_);
		return queue_.empty();
	}

	void abort() override {
		{
			std::lock_guard<std::mutex> lock(mtx_);
			is_aborted_ = true;
		}
		cv_.notify_all();
	}
};

#endif /* THREAD_SAFE_QUEUE_HPP_ */

