#ifndef I_MESSAGE_QUEUE_HPP_
#define I_MESSAGE_QUEUE_HPP_

template <typename T>
class IMessageQueue {
public:
	virtual ~IMessageQueue() = default;

	virtual bool push(const T& item) = 0;
	virtual bool pop(T& item) = 0;
	virtual bool is_empty() const = 0;
	virtual void abort() {}
};

#endif /* I_MESSAGE_QUEUE_HPP_ */
