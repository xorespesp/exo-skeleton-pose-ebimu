#pragma once
#include <Windows.h>
#include <atomic>
#include <mutex>

namespace utils
{
	class spin_lock {
		mutable std::atomic_flag _flag;

	public:
		spin_lock() { _flag.clear(); }
		virtual ~spin_lock() { }

		spin_lock(const spin_lock&) = delete;
		spin_lock& operator= (const spin_lock&) = delete;

		void lock() const noexcept { while (_flag.test_and_set(std::memory_order_acquire)); }
		void unlock() const noexcept { _flag.clear(std::memory_order_release); }
	};

} // namespace