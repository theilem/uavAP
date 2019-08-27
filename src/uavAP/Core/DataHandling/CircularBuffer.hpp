/*
 * CircularBuffer.hpp
 *
 *  Created on: Aug 13, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_DATAHANDLING_CIRCULARBUFFER_HPP_
#define UAVAP_CORE_DATAHANDLING_CIRCULARBUFFER_HPP_
#include <uavAP/Core/LockTypes.h>
#include <cstddef>
#include <memory>
#include <vector>

template<typename T>
class CircularBuffer
{
public:

	using value_type = T;

	explicit
	CircularBuffer(size_t size);

	void
	put(const value_type& item);

	value_type
	get();

	value_type
	getTail() const;

	std::vector<T>
	getAll() const;

	void
	reset();

	bool
	empty() const;

	bool
	full() const;

	size_t
	capacity() const;

	size_t
	size() const;

private:

	mutable Mutex mutex_;
	std::unique_ptr<value_type[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t maxSize_;
	bool full_ = 0;
};

template<typename T>
inline
CircularBuffer<T>::CircularBuffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])), maxSize_(size)
{
}

template<typename T>
inline void
CircularBuffer<T>::put(const value_type& item)
{
	LockGuard lock(mutex_);

	buf_[head_] = item;

	if (full_)
	{
		tail_ = (tail_ + 1) % maxSize_;
	}

	head_ = (head_ + 1) % maxSize_;

	full_ = head_ == tail_;
}

template<typename T>
inline typename CircularBuffer<T>::value_type
CircularBuffer<T>::get()
{
	LockGuard lock(mutex_);

	if (empty())
	{
		return T();
	}

	//Read data and advance the tail (we now have a free space)
	auto val = buf_[tail_];
	full_ = false;
	tail_ = (tail_ + 1) % maxSize_;

	return val;
}

template<typename T>
inline void
CircularBuffer<T>::reset()
{
	LockGuard lock(mutex_);
	head_ = tail_;
	full_ = false;
}

template<typename T>
inline bool
CircularBuffer<T>::empty() const
{
	return (!full_ && (head_ == tail_));
}

template<typename T>
inline bool
CircularBuffer<T>::full() const
{
	return full_;
}

template<typename T>
inline size_t
CircularBuffer<T>::capacity() const
{
	return maxSize_;
}

template<typename T>
inline size_t
CircularBuffer<T>::size() const
{
	size_t size = maxSize_;

	if (!full_)
	{
		if (head_ >= tail_)
		{
			size = head_ - tail_;
		}
		else
		{
			size = maxSize_ + head_ - tail_;
		}
	}

	return size;
}

template<typename T>
inline typename CircularBuffer<T>::value_type
CircularBuffer<T>::getTail() const
{
	LockGuard lock(mutex_);

	if (empty())
	{
		return T();
	}

	return buf_[tail_];
}

template<typename T>
inline std::vector<T>
CircularBuffer<T>::getAll() const
{
	LockGuard lock(mutex_);

	if (empty())
		return std::vector<T>();

	std::vector<value_type> vec(size(), T());
	if (head_ <= tail_)
	{
		memcpy(static_cast<void*>(vec[0]), static_cast<void*>(&buf_[tail_]), (maxSize_ - tail_ - 1) * sizeof(T));
		memcpy(static_cast<void*>(vec[(maxSize_ - tail_ - 1)]), static_cast<void*>(&buf_[0]), (head_ - 1) * sizeof(T));
	}
	else
	{
		memcpy(static_cast<void*>(vec[0]), static_cast<void*>(&buf_[tail_]), (head_ - tail_ - 1) * sizeof(T));
	}

	return vec;
}

#endif /* UAVAP_CORE_DATAHANDLING_CIRCULARBUFFER_HPP_ */
