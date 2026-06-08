#ifndef BUFFER_VIEW_HPP_
#define BUFFER_VIEW_HPP_

#include <cstdint>
#include <cstring>

class BufferView {
private:
	uint8_t* const data_;
	const size_t size_;
	size_t offset_;

public:
	BufferView(uint8_t* buffer, size_t size);

	template <typename T>
	bool write(T value) {
		if (offset_ + sizeof(T) > size_) return false;
		for (size_t i = 0; i < sizeof(T); ++i) {
			data_[offset_ + i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
		}
		offset_ += sizeof(T);
		return true;
	}

	template <typename T>
	bool read(T& value) {
		if (offset_ + sizeof(T) > size_) return false;
		value = 0;
		for (size_t i = 0; i < sizeof(T); ++i) {
			value |= (static_cast<T>(data_[offset_ + i]) << (i * 8));
		}
		offset_ += sizeof(T);
		return true;
	}

	bool writeFloat(float value);
	bool readFloat(float& value);
	size_t getOffset() const;
};

#endif /* BUFFER_VIEW_HPP_ */
