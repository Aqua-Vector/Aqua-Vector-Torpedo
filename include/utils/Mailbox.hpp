#ifndef MAILBOX_HPP_
#define MAILBOX_HPP_

#include <mutex>
#include <atomic>

template <typename T>
class Mailbox {
private:
	T data_;
	std::mutex mtx_;
	std::atomic<uint64_t> last_update_ms_{0};

public:
	Mailbox() = default;

	Mailbox(const Mailbox&) = delete;
	Mailbox& operator=(const Mailbox&) = delete;

	void update(const T& new_data, uint64_t timestamp) {
		std::lock_guard<std::mutex> lock(mtx_);
		data_ = new_data;
		last_update_ms_.store(timestamp, std::memory_order_release);
	}

	bool fetch(T& out_data, uint64_t& out_timestamp) {
		std::lock_guard<std::mutex> lock(mtx_);
		out_data = data_;
		out_timestamp = last_update_ms_.load(std::memory_order_acquire);

		return (out_timestamp != 0);
	}

	uint64_t getLastUpdateTime() const {
		return last_update_ms_.load(std::memory_order_acquire);
	}
};

#endif /* MAILBOX_HPP_ */
