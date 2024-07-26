#pragma once
// for boost.asio
#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT  _WIN32_WINNT_WIN10
#endif
#if !defined(_WINSOCK_DEPRECATED_NO_WARNINGS)
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#define BOOST_ASIO_HAS_IOCP 1
#define BOOST_ASIO_HAS_SERIAL_PORT 1
#include <boost/asio.hpp>

#include "../utils/logger.hh"
#include <memory>
#include <vector>
#include <thread>
#include <mutex>

namespace core
{
	class io_context_pool final
	{
	private:
		using this_type = io_context_pool;
		using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

		template <typename T>
		using shared_ptr_vector = std::vector<std::shared_ptr<T>>;

	private:
		shared_ptr_vector<boost::asio::io_context> _io_contexts; // The pool of io_contexts.
		shared_ptr_vector<work_guard_type> _work_guards; // The work-tracking executors that keep the io_contexts running.
		std::size_t _next_io_context_index; // The next io_context to use for a connection.
		std::vector<std::thread> _threads;
		mutable std::recursive_mutex _mtx;

	public:
		io_context_pool(std::size_t pool_size = 0)
			: _next_io_context_index{ 0 }
		{
			LOG_TRACE("%s() ENTER", __func__);

			if (!pool_size) {
				pool_size = std::thread::hardware_concurrency() * 2;
			}

			// Give all the io_contexts work to do so that their run() functions will not
			// exit until they are explicitly stopped.
			for (std::size_t i = 0; i < pool_size; ++i) {
				_io_contexts.emplace_back(std::make_shared<boost::asio::io_context>());
				_work_guards.emplace_back(std::make_shared<work_guard_type>(_io_contexts.back()->get_executor()));
			}

			this->run_async();

			LOG_TRACE("%s() LEAVE", __func__);
		}

		~io_context_pool() {
			LOG_TRACE("%s() ENTER", __func__);

			this->stop();

			LOG_TRACE("%s() LEAVE", __func__);
		}

		io_context_pool(const this_type&) = delete;
		this_type& operator=(const this_type&) = delete;

		bool is_running() const noexcept {
			std::scoped_lock lk{ _mtx };
			return !_threads.empty();
		}

		void run_async()
		{
			std::scoped_lock lk{ _mtx };
			if (!this->is_running())
			{
				// Create a pool of threads to run all of the io_contexts.
				for (auto io_ctx_ptr : _io_contexts)
				{
					_threads.emplace_back(
						[io_ctx_ptr]() -> void
						{
							LOG_TRACE("----- IO THREAD(0x%X) BEGIN", ::GetCurrentThreadId());

							try {
								io_ctx_ptr->run();
							} catch (const std::exception& e) {
								LOG_CRITICAL("IO THREAD EXEPCTION -> %s", e.what());
							}

							LOG_TRACE("----- IO THREAD(0x%X) END", ::GetCurrentThreadId());
						});
				}
			}
		}

		void stop()
		{
			std::scoped_lock lk{ _mtx };
			if (this->is_running())
			{
				// Explicitly stop all io_contexts.
				for (auto io_ctx : _io_contexts) {
					io_ctx->stop(); // That's OK, io_context::stop is thread-safe
				}

				// Wait for all threads in the pool to exit.
				for (auto& thread : _threads) {
					thread.join();
				}
				_threads.clear();
			}
		}

		boost::asio::io_context& get_io_context()
		{
			std::scoped_lock lk{ _mtx };

			boost::asio::io_context& io_ctx = *_io_contexts[_next_io_context_index];

			// Use a round-robin scheme to choose the next io_context to use.
			if (++_next_io_context_index == _io_contexts.size()) {
				_next_io_context_index = 0;
			}

			return io_ctx;
		}

		template <class _TaskFunc>
		void post_task(_TaskFunc&& task_fn) {
			auto& io_ctx = this->get_io_context();
			io_ctx.post(std::forward<_TaskFunc>(task_fn));
		}

	}; // class

} // namespace