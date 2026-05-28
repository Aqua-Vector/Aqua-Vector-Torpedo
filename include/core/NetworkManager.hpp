#ifndef NETWORK_MANAGER_HPP_
#define NETWORK_MANAGER_HPP_

#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include <iostream>
#include <cstdio>
#include "CommInterfaces.hpp"
#include "IMessageQueue.hpp"
#include "StaticRingBuffer.hpp"

/**
 * @brief Base interface for NetworkManager
 */
class INetworkManager {
public:
	virtual ~INetworkManager() = default;
	virtual bool start() = 0;
	virtual void stop() = 0;
};

template <typename TxPacket>
class ITxNetworkManager : public INetworkManager {
public:
	virtual bool send(const TxPacket& packet) = 0;
};

template <typename ParserT, typename LinkT, typename TxPacket>
class NetworkManager : public ITxNetworkManager<TxPacket> {
private:
	LinkT& link_;
	ParserT& parser_;
	IMessageQueue<TxPacket>& tx_queue_;
	StaticRingBuffer<uint8_t, 2048> rx_fifo_;

	std::thread rx_thread_;
	std::thread tx_thread_;
	std::atomic<bool> is_running_;

	void rxWorkerLoop() {
		uint8_t chunk[256];
		while (is_running_.load(std::memory_order_acquire)) {
			ssize_t bytes_read = link_.receive(chunk, sizeof(chunk));
			uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count();

			if (bytes_read > 0) {
				for (ssize_t i = 0; i < bytes_read; ++i) {
					if (!rx_fifo_.push(chunk[i])) break;
				}
			}

			uint8_t data;
			while (rx_fifo_.pop(data)) {
				parser_.parseByte(data, now);
			}

			if (bytes_read <= 0) {
				// 데이터가 없을 때만 짧게 대기하여 CPU 부하 경감
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
					// 지연 요소를 최소화하여 즉시 송신
					link_.send(tx_buf, len);
				}
			} else {
				// 보낼 게 없을 때만 대기
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}

public:
	NetworkManager(LinkT& link, ParserT& parser, IMessageQueue<TxPacket>& tx_queue)
		: link_(link), parser_(parser), tx_queue_(tx_queue), is_running_(false) {}

	~NetworkManager() { stop(); }

	bool start() override {
		if (is_running_.load(std::memory_order_acquire)) return true;
		if (!link_.initialize()) return false;
		is_running_.store(true, std::memory_order_release);
		rx_thread_ = std::thread(&NetworkManager::rxWorkerLoop, this);
		tx_thread_ = std::thread(&NetworkManager::txWorkerLoop, this);
		return true;
	}

	void stop() override {
		if (!is_running_.load(std::memory_order_acquire)) return;
		is_running_.store(false, std::memory_order_release);
		tx_queue_.abort();
		if (rx_thread_.joinable()) rx_thread_.join();
		if (tx_thread_.joinable()) tx_thread_.join();
		link_.close();
	}

	bool send(const TxPacket& packet) override {
		if (!is_running_.load(std::memory_order_acquire)) return false;
		return tx_queue_.push(packet);
	}
};

#endif /* NETWORK_MANAGER_HPP_ */
