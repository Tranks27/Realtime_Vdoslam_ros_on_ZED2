/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeQueue.h
 * @brief  Thread Safe Queue with shutdown/resume functionality.
 * @author Antoni Rosinol
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>

// #include <glog/logging.h>



template <typename T>
class ThreadsafeQueueBase {
	protected:
		typedef std::queue<std::shared_ptr<T>> InternalQueue;
		explicit ThreadsafeQueueBase();

		virtual ~ThreadsafeQueueBase() = default;

	public:


		/** \brief Push by value. Returns false if the queue has been shutdown.
		 * Not optimal, since it will make two move operations.
		 * But it does the job: see Item 41 Effective Modern C++
		 *
		 * Alternatives:
		 *  Using both:
		 *  - push(const T&)
		 *  - with push(T&& ) rvalue to the queue using move semantics.
		 * Since there is no type deduction, T&& is NOT a universal
		 * reference (typename T is not at the level of the push function).
		 * Problem: virtual keyword will make push(T&&) be discarded for
		 * push(const T&), non-copyable things will compile-complain.
		 *
		 */
		virtual bool push(T new_value) = 0;

		/** * @brief pushBlockingIfFull pushes a value into the queue only if the
		 * queue is not filled with more than a given maximum size.
		 * @param new_value new value to add to the queue
		 * @param max_queue_size if the queue is filled with more than max_queue_size
		 * messages, it will not push to the queue, and it will wait for a consumer
		 * to remove messages.
		 * @return false if the queue has been shutdown
		 */
		virtual bool pushBlockingIfFull(T new_value, size_t max_queue_size = 10u) = 0;

		/** \brief Pop value. Waits for data to be available in the queue.
		 * Returns false if the queue has been shutdown.
		 */
		virtual bool popBlocking(T& value) = 0;

		/** \brief Pop value. Waits for data to be available in the queue.
		 * If the queue has been shutdown, it returns a null shared_ptr.
		 */
		virtual std::shared_ptr<T> popBlocking() = 0;

		virtual bool popBlockingWithTimeout(T& value, size_t duration_ms) = 0;

		/** \brief Pop without blocking, just checks once if the queue is empty.
		 * Returns true if the value could be retrieved, false otherwise.
		 */
		virtual bool pop(T& value) = 0;

		/** \brief Pop without blocking, just checks once if the queue is empty.
		 * Returns a shared_ptr to the value retrieved.
		 * If the queue is empty or has been shutdown,
		 * it returns a null shared_ptr.
		 */
		virtual std::shared_ptr<T> pop() = 0;

		int size() {
			return data_queue_.size();
		}

		void shutdown() {
			// VLOG(1) << "Shutting down queue: " << queue_id_;
			std::unique_lock<std::mutex> mlock(mutex_);
			// Even if the shared variable is atomic, it must be modified under the
			// mutex in order to correctly publish the modification to the waiting
			// threads.
			shutdown_ = true;
			mlock.unlock();
			data_cond_.notify_all();
		}

		inline bool isShutdown() { return shutdown_; }

		void resume() {
			std::unique_lock<std::mutex> mlock(mutex_);
			// Even if the shared variable is atomic, it must be modified under the
			// mutex in order to correctly publish the modification to the waiting
			// threads.
			shutdown_ = false;
			mlock.unlock();
			data_cond_.notify_all();
		}

		/** \brief Checks if the queue is empty.
		 * the state of the queue might change right after this query.
		 */
		bool empty() const {
			std::lock_guard<std::mutex> lk(mutex_);
			return data_queue_.empty();
		}

		/** \brief Checks if the queue is shutdown.
		 * the state of the queue might change right after this query.
		 */
		bool isShutdown() const {
			std::lock_guard<std::mutex> lk(mutex_);
			return shutdown_;
		}



	protected:
		mutable std::mutex mutex_;  //! mutable for empty() and copy-constructor.
		InternalQueue data_queue_;
		std::condition_variable data_cond_;
		std::atomic_bool shutdown_;  //! flag for signaling queue shutdown.
};

template <typename T>
class ThreadsafeQueue : public ThreadsafeQueueBase<T> {
 	public:
		using TQB = ThreadsafeQueueBase<T>;
		explicit ThreadsafeQueue();
		virtual ~ThreadsafeQueue() = default;

		/** \brief Push by value. Returns false if the queue has been shutdown.
		 * Not optimal, since it will make two move operations.
		 * But it does the job: see Item 41 Effective Modern C++
		 *
		 * Alternatives:
		 *  Using both:
		 *  - push(const T&)
		 *  - with push(T&& ) rvalue to the queue using move semantics.
		 * Since there is no type deduction, T&& is NOT a universal
		 * reference (typename T is not at the level of the push function).
		 * Problem: virtual keyword will make push(T&&) be discarded for
		 * push(const T&), non-copyable things will compile-complain.
		 *
		 */
		bool push(T new_value) override;

		/** * @brief pushBlockingIfFull pushes a value into the queue only if the
		 * queue is not filled with more than a given maximum size.
		 * @param new_value new value to add to the queue
		 * @param max_queue_size if the queue is filled with more than max_queue_size
		 * messages, it will not push to the queue, and it will wait for a consumer
		 * to remove messages.
		 * @return false if the queue has been shutdown
		 */
		bool pushBlockingIfFull(T new_value, size_t max_queue_size = 10u) override;

		/** \brief Pop value. Waits for data to be available in the queue.
		 * Returns false if the queue has been shutdown.
		 */
		// TODO(Toni): add a timer to avoid waiting forever...
		bool popBlocking(T& value) override;

		/** \brief Pop value. Waits for data to be available in the queue.
		 * If the queue has been shutdown, it returns a null shared_ptr.
		 */
		std::shared_ptr<T> popBlocking() override;

		/**
		 * @brief popBlockingUpToTime Same as Pop blocking, but further returns early
		 * if the given duration has passed...
		 * @param value Returned value
		 * @param duration_ms Time to wait for a msg [in milliseconds]
		 * @return Returns false if the queue has been shutdown or if it was timeout.
		 */
		bool popBlockingWithTimeout(T& value, size_t duration_ms) override;

		/** \brief Pop without blocking, just checks once if the queue is empty.
		 * Returns true if the value could be retrieved, false otherwise.
		 */
		bool pop(T& value) override;

		/** \brief Pop without blocking, just checks once if the queue is empty.
		 * Returns a shared_ptr to the value retrieved.
		 * If the queue is empty or has been shutdown,
		 * it returns a null shared_ptr.
		 */
		std::shared_ptr<T> pop() override;

	private:
		using TQB::data_cond_;
		using TQB::data_queue_;
		using TQB::mutex_;
		using TQB::shutdown_;

};


template <typename T>
ThreadsafeQueueBase<T>::ThreadsafeQueueBase()
    : mutex_(),
      data_queue_(),
      data_cond_(),
      shutdown_(false) {}

template <typename T>
ThreadsafeQueue<T>::ThreadsafeQueue()
    : ThreadsafeQueueBase<T>() {}

template <typename T>
bool ThreadsafeQueue<T>::push(T new_value) {
	if (shutdown_) return false;  // atomic, no lock needed.
	std::shared_ptr<T> data(std::make_shared<T>(std::move(new_value)));
	std::unique_lock<std::mutex> lk(mutex_);
	data_queue_.push(data);
	size_t queue_size = data_queue_.size();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	// Thread-safe so doesn't need external mutex
	return true;
}

template <typename T>
bool ThreadsafeQueue<T>::pushBlockingIfFull(T new_value,
                                            size_t max_queue_size) {
	if (shutdown_) return false;  // atomic, no lock needed.
	std::shared_ptr<T> data(std::make_shared<T>(std::move(new_value)));
	std::unique_lock<std::mutex> lk(mutex_);
	// Wait until the queue has space or shutdown requested.
	data_cond_.wait(lk, [this, max_queue_size] {
		return data_queue_.size() < max_queue_size || shutdown_;
	});
	if (shutdown_) return false;
	data_queue_.push(data);
	size_t queue_size = data_queue_.size();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	// Thread-safe so doesn't need external mutex.
	return true;
}

template <typename T>
bool ThreadsafeQueue<T>::popBlocking(T& value) {
	std::unique_lock<std::mutex> lk(mutex_);
	// Wait until there is data in the queue or shutdown requested.
	data_cond_.wait(lk, [this] { return !data_queue_.empty() || shutdown_; });
	// Return false in case shutdown is requested.
	if (shutdown_) return false;
	value = std::move(*data_queue_.front());
	data_queue_.pop();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	return true;
}

template <typename T>
std::shared_ptr<T> ThreadsafeQueue<T>::popBlocking() {
	std::unique_lock<std::mutex> lk(mutex_);
	data_cond_.wait(lk, [this] { return !data_queue_.empty() || shutdown_; });
	if (shutdown_) return std::shared_ptr<T>(nullptr);
	std::shared_ptr<T> result = data_queue_.front();
	data_queue_.pop();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	return result;
}

template <typename T>
bool ThreadsafeQueue<T>::popBlockingWithTimeout(T& value, size_t duration_ms) {
	std::unique_lock<std::mutex> lk(mutex_);
	// Wait until there is data in the queue, shutdown is requested, or
	// the given time is elapsed...
	data_cond_.wait_for(lk, std::chrono::milliseconds(duration_ms), [this] {
	return !data_queue_.empty() || shutdown_;
	});
	// Return false in case shutdown is requested or the queue is empty (in which
	// case, a timeout has happened).
	if (shutdown_ || data_queue_.empty()) return false;
	value = std::move(*data_queue_.front());
	data_queue_.pop();
	return true;
}

template <typename T>
bool ThreadsafeQueue<T>::pop(T& value) {
	if (shutdown_) return false;
	std::unique_lock<std::mutex> lk(mutex_);
	if (data_queue_.empty()) return false;
	value = std::move(*data_queue_.front());
	data_queue_.pop();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	return true;
}

template <typename T>
std::shared_ptr<T> ThreadsafeQueue<T>::pop() {
	if (shutdown_) return std::shared_ptr<T>(nullptr);
	std::unique_lock<std::mutex> lk(mutex_);
	if (data_queue_.empty()) return std::shared_ptr<T>(nullptr);
	std::shared_ptr<T> result = data_queue_.front();
	data_queue_.pop();
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	return result;
}


/**
 * @brief Same as threadsafe queue but has a maximum size allowance.
 * If the size of the queue exceeds the max size when pushed, the oldest object will
 * removed from the queue and the new object pushed on. This differs 
 * from the default ThreadSafe Queue behaviour which will wait till 
 * size < max size
 * 
 * @tparam T 
 */
template<typename T, int N>
class ThreadSafeMaxQueue : public ThreadsafeQueue<T> {
	public:
		using TQ = ThreadsafeQueue<T>;
		explicit ThreadSafeMaxQueue();
		virtual ~ThreadSafeMaxQueue() = default;

		bool push(T new_value) override;

	private:
		using TQ::data_cond_;
		using TQ::data_queue_;
		using TQ::mutex_;
		using TQ::shutdown_;



};

template<typename T, int N>
ThreadSafeMaxQueue<T, N>::ThreadSafeMaxQueue()
	:	ThreadsafeQueue<T>() {}


template <typename T, int N>
bool ThreadSafeMaxQueue<T, N>::push(T new_value) {
	if (shutdown_) return false;
	std::shared_ptr<T> data(std::make_shared<T>(std::move(new_value)));
	std::unique_lock<std::mutex> lk(mutex_);

	size_t queue_size = data_queue_.size();
	if (queue_size > N) {
		data_queue_.pop();
	}
	data_queue_.push(data);
	lk.unlock();  // Unlock before notify.
	data_cond_.notify_one();
	// Thread-safe so doesn't need external mutex
	return true;
}


