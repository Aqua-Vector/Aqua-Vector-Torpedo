#ifndef NETWORK_MANAGER_HPP_
#define NETWORK_MANAGER_HPP_

#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include "CommInterfaces.hpp"
#include "IMessageQueue.hpp"
#include "StaticRingBuffer.hpp"

template <typename ParserT, typename LinkT, typename TxPacket>
class NetworkManager {
private:
	LinkT& link_;
	ParserT& parser_;
	IMessageQueue<TxPacket>& tx_queue_;
	StaticRingBuffer<uint8_t, 2048> rx_fifo_;

	std::thread rx_thread_;
	std::thread tx_thread_;
	std::atomic<bool> is_running_;

	void rxWorkerLoop() {
		uint8_t chunk[64];

		while (is_running_.load(std::memory_order_acquire)) {
			size_t bytes_read = link_.receive(chunk, sizeof(chunk));
			if (bytes_read > 0) {
				// We don't have an easy way to check debug_mode_ here without changing interface,
				// but for hardware test we'll just push it. 
				// GenericParser will handle the actual printing if debug is on.
				for (size_t i = 0; i < bytes_read; ++i) {
					rx_fifo_.push(chunk[i]);
				}
			}

			uint8_t data;
			uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count();

			while (rx_fifo_.pop(data)) {
				parser_.parseByte(data, now);
			}

			if (bytes_read == 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}

	void txWorkerLoop() {
		TxPacket packet;
		uint8_t tx_buf[ParserT::ProtocolPolicy::MAX_PAYLOAD_SIZE + 8];

		while (is_running_.load(std::memory_order_acquire)) {
			if (tx_queue_.pop(packet)) {
				size_t len = parser_.serialize(packet, tx_buf, sizeof(tx_buf));
				if (len > 0) {
					link_.send(tx_buf, len);
				}
			}
		}
	}

public:
	NetworkManager(LinkT& link, ParserT& parser, IMessageQueue<TxPacket>& tx_queue)
		: link_(link), parser_(parser), tx_queue_(tx_queue), is_running_(false) {}

	~NetworkManager() { stop(); }

	bool start() {
		if (is_running_.load(std::memory_order_acquire)) return true;
		if (!link_.initialize()) return false;

		is_running_.store(true, std::memory_order_release);
		rx_thread_ = std::thread(&NetworkManager::rxWorkerLoop, this);
		tx_thread_ = std::thread(&NetworkManager::txWorkerLoop, this);

		return true;
	}

	void stop() {
		if (!is_running_.load(std::memory_order_acquire)) return;
		is_running_.store(false, std::memory_order_release);

		tx_queue_.abort();

		if (rx_thread_.joinable()) rx_thread_.join();
		if (tx_thread_.joinable()) tx_thread_.join();

		link_.close();
	}

	bool send(const TxPacket& packet) {
		if (!is_running_.load(std::memory_order_acquire)) return false;
		return tx_queue_.push(packet);
	}
};

#endif /* NETWORK_MANAGER_HPP_ */
