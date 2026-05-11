#include "BufferView.hpp"
#include <cstring>

BufferView::BufferView(uint8_t* buffer, size_t size)
	: data_(buffer), size_(size), offset_(0) {}

bool BufferView::writeFloat(float value) {
	uint32_t temp;
	std::memcpy(&temp, &value, sizeof(float));
	return write<uint32_t>(temp);
}

bool BufferView::readFloat(float& value) {
	uint32_t temp;
	if (!read<uint32_t>(temp)) return false;
	std::memcpy(&value, &temp, sizeof(float));
	return true;
}

size_t BufferView::getOffset() const {
	return offset_;
}

