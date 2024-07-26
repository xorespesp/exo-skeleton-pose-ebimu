#pragma once
#include "c_format.hh"
#include "codecvt.hh"
#include "mutex.hh"

#include <filesystem>

#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/fmt/ostr.h> // support for user defined types
#include <spdlog/sinks/msvc_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/callback_sink.h>


#define _CALL_LOGGER(LEVEL, ...) ::utils::logger::instance().log(::utils::logger::src_loc{ __FILE__, __LINE__, __FUNCTION__ }, LEVEL, __VA_ARGS__)

#define LOG_TRACE(...)    _CALL_LOGGER(::utils::logger::level::trace, __VA_ARGS__)
#define LOG_DEBUG(...)    _CALL_LOGGER(::utils::logger::level::debug, __VA_ARGS__)
#define LOG_INFO(...)     _CALL_LOGGER(::utils::logger::level::info, __VA_ARGS__)
#define LOG_WARN(...)     _CALL_LOGGER(::utils::logger::level::warn, __VA_ARGS__)
#define LOG_ERROR(...)    _CALL_LOGGER(::utils::logger::level::err, __VA_ARGS__)
#define LOG_CRITICAL(...) _CALL_LOGGER(::utils::logger::level::critical, __VA_ARGS__)


namespace utils
{
    class logger final {
    public:
        enum class level {
            trace = SPDLOG_LEVEL_TRACE,
            debug = SPDLOG_LEVEL_DEBUG,
            info = SPDLOG_LEVEL_INFO,
            warn = SPDLOG_LEVEL_WARN,
            err = SPDLOG_LEVEL_ERROR,
            critical = SPDLOG_LEVEL_CRITICAL,
        };

        struct src_loc {
            constexpr src_loc() = default;
            constexpr src_loc(const char* filename_in, int line_in, const char* funcname_in)
                : filename{ filename_in }
                , line{ line_in }
                , funcname{ funcname_in }
            {}

            constexpr bool empty() const noexcept { return line == 0; }

            const char* filename{ nullptr };
            int line{ 0 };
            const char* funcname{ nullptr };
        };

        class init_option {
        private:
            std::string _logger_name{ "logger" };
            level _logger_level{ level::trace };
            bool _is_async_logger{ false };
            size_t _asnyc_logger_q_size{ 0 }, _async_logger_thread_count{ 0 };
            std::vector<spdlog::sink_ptr> _logger_sinks{};

        public:
            init_option() = default;

            init_option& set_logger_name(const std::string_view new_name) {
                _logger_name = new_name;
                return *this;
            }

            init_option& set_logger_level(const level lv) {
                _logger_level = lv;
                return *this;
            }

            init_option& enable_async_mode(const size_t q_size = 16 * 1024, const size_t thread_count = 1) {
                _is_async_logger = true;
                _asnyc_logger_q_size = q_size;
                _async_logger_thread_count = thread_count;
                return *this;
            }

            init_option& enable_file_logging(const std::filesystem::path& file_path, const level lv = level::trace) {
                auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_path.string(), true);
                file_sink->set_pattern("%Y-%m-%d %H:%M:%S.%f %z | %n | ---%L--- | TID %t | %s:%#@%! | %v");
                file_sink->set_level(static_cast<spdlog::level::level_enum>(lv));

                _logger_sinks.push_back(file_sink);
                return *this;
            }

            init_option& enable_stdout_logging(const level lv = level::trace) {
                auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                stdout_sink->set_pattern("%^%H:%M:%S.%f | %n | TID %t | %s:%# | %v%$");
                stdout_sink->set_level(static_cast<spdlog::level::level_enum>(lv));

                _logger_sinks.push_back(stdout_sink);
                return *this;
            }

            init_option& enable_callback_logging(const level lv = level::trace) {
                auto callback_sink = std::make_shared<spdlog::sinks::callback_sink_mt>(
                    [](const spdlog::details::log_msg& /*msg*/) {
                        // for example you can be notified by sending an email to yourself
                    });
                callback_sink->set_pattern("%H:%M:%S.%f | %n | TID %t | %s:%# | %v");
                callback_sink->set_level(static_cast<spdlog::level::level_enum>(lv));

                _logger_sinks.push_back(callback_sink);
                return *this;
            }

            friend class logger;
        };

    private:
        utils::spin_lock _init_lock;
        std::atomic_bool _init_flag = false;
        std::shared_ptr<spdlog::logger> _logger_obj;

    private:
        logger() = default;
        ~logger() = default;

        logger(const logger&) = delete;
        logger& operator= (const logger&) = delete;

    public:
        static logger& instance() {
            static logger obj{};
            return obj;
        }

        bool is_inited() const noexcept {
            return _init_flag;
        }

        void init(init_option opt = init_option{})
        {
            std::scoped_lock lck{ _init_lock };
            if (this->is_inited()) { return; }

            // Add default sink (DebugOutputString)
            auto msvc_sink = std::make_shared<spdlog::sinks::msvc_sink_mt>(false);
            msvc_sink->set_pattern("%H:%M:%S.%f | %n | ---%L--- | TID %t | %s:%# | %v");
            msvc_sink->set_level(spdlog::level::trace);
            opt._logger_sinks.push_back(msvc_sink);

            if (opt._is_async_logger)
            {
                spdlog::init_thread_pool(opt._asnyc_logger_q_size, opt._async_logger_thread_count);
                _logger_obj = std::make_shared<spdlog::async_logger>(
                    opt._logger_name,
                    opt._logger_sinks.begin(),
                    opt._logger_sinks.end(),
                    spdlog::thread_pool(),
                    spdlog::async_overflow_policy::block
                );
            }
            else
            {
                _logger_obj = std::make_shared<spdlog::logger>(
                    opt._logger_name,
                    opt._logger_sinks.begin(),
                    opt._logger_sinks.end()
                );
            }

            _logger_obj->set_level(static_cast<spdlog::level::level_enum>(opt._logger_level));
            _logger_obj->flush_on(static_cast<spdlog::level::level_enum>(level::err));

            // periodically flush all *registered* loggers every 3 seconds:
            // warning: only use if all your loggers are thread-safe ("_mt" loggers)
            spdlog::flush_every(std::chrono::seconds(3));

            _init_flag = true;
        }

        void deinit()
        {
            std::scoped_lock lck{ _init_lock };
            if (!this->is_inited()) { return; }

            // spdlog::shutdown()
            //     Release all spdlog resources, and drop all loggers in the registry.
            //     This is optional (only mandatory if using windows + async log).
            // 
            // NOTE:
            //     There is a bug in VS runtime that cause the application dead-lock when it exits.
            //     If you use async logging, please make sure to call spdlog::shutdown() before main() exit.
            //     http://stackoverflow.com/questions/10915233/stdthreadjoin-hangs-if-called-after-main-exits-when-using-vs2012
            spdlog::shutdown();
            _logger_obj.reset(); // `std::shared_ptr` is thread-unsafe; MUST call `reset()` after spdlog shutdown

            _init_flag = false;
        }

        void set_level(const level lv)
        {
            _logger_obj->set_level(static_cast<spdlog::level::level_enum>(lv));
        }

        void log(
            const src_loc src,
            const level lv,
            const std::string_view msg)
        {
            if (!this->is_inited()) { return; }

            _logger_obj->log(
                spdlog::source_loc{ src.filename, src.line, src.funcname },
                static_cast<spdlog::level::level_enum>(lv),
                msg.data()
            );
        }

        template <typename... _Args>
        void log(
            const src_loc src,
            const level lv,
            const std::string& fmt,
            _Args&&... args)
        {
            if (!this->is_inited()) { return; }

            auto msg = utils::string::c_format(fmt, std::forward<_Args>(args)...);
            _logger_obj->log(
                spdlog::source_loc{ src.filename, src.line, src.funcname },
                static_cast<spdlog::level::level_enum>(lv),
                msg.c_str()
            );
        }

        void log(
            const src_loc src,
            const level lv,
            const std::wstring_view wmsg)
        {
            if (!this->is_inited()) { return; }

            _logger_obj->log(
                spdlog::source_loc{ src.filename, src.line, src.funcname },
                static_cast<spdlog::level::level_enum>(lv),
#if defined(SPDLOG_WCHAR_TO_UTF8_SUPPORT)
                wmsg
#else  // ^^^ SPDLOG_WCHAR_TO_UTF8_SUPPORT ^^^ / vvv !SPDLOG_WCHAR_TO_UTF8_SUPPORT vvv
                utils::string::utf16_to_utf8(wmsg).c_str()
#endif // ^^^ !SPDLOG_WCHAR_TO_UTF8_SUPPORT ^^^
            );
        }

        template <typename... _Args>
        void log(
            const src_loc src,
            const level lv,
            const std::wstring& wfmt,
            _Args&&... args)
        {
            if (!this->is_inited()) { return; }

            auto wmsg = utils::string::c_wformat(wfmt, std::forward<_Args>(args)...);
            _logger_obj->log(
                spdlog::source_loc{ src.filename, src.line, src.funcname },
                static_cast<spdlog::level::level_enum>(lv),
#if defined(SPDLOG_WCHAR_TO_UTF8_SUPPORT)
                wmsg.c_str()
#else  // ^^^ SPDLOG_WCHAR_TO_UTF8_SUPPORT ^^^ / vvv !SPDLOG_WCHAR_TO_UTF8_SUPPORT vvv
                utils::string::utf16_to_utf8(wmsg).c_str()
#endif // ^^^ !SPDLOG_WCHAR_TO_UTF8_SUPPORT ^^^
            );
        }

        void log_trace(std::string_view msg) { this->log(src_loc{}, level::trace, msg); }
        void log_debug(std::string_view msg) { this->log(src_loc{}, level::debug, msg); }
        void log_info(std::string_view msg) { this->log(src_loc{}, level::info, msg); }
        void log_warn(std::string_view msg) { this->log(src_loc{}, level::warn, msg); }
        void log_err(std::string_view msg) { this->log(src_loc{}, level::err, msg); }
        void log_critical(std::string_view msg) { this->log(src_loc{}, level::critical, msg); }

        void log_trace(std::wstring_view wmsg) { this->log(src_loc{}, level::trace, wmsg); }
        void log_debug(std::wstring_view wmsg) { this->log(src_loc{}, level::debug, wmsg); }
        void log_info(std::wstring_view wmsg) { this->log(src_loc{}, level::info, wmsg); }
        void log_warn(std::wstring_view wmsg) { this->log(src_loc{}, level::warn, wmsg); }
        void log_err(std::wstring_view wmsg) { this->log(src_loc{}, level::err, wmsg); }
        void log_critical(std::wstring_view wmsg) { this->log(src_loc{}, level::critical, wmsg); }

        template <typename... _Args> void log_trace(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::trace, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_debug(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::debug, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_info(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::info, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_warn(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::warn, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_err(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::err, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_critical(const std::string& fmt, _Args&&... args) { this->log(src_loc{}, level::critical, fmt, std::forward<_Args>(args)...); }

        template <typename... _Args> void log_trace(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::trace, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_debug(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::debug, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_info(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::info, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_warn(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::warn, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_err(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::err, fmt, std::forward<_Args>(args)...); }
        template <typename... _Args> void log_critical(const std::wstring& fmt, _Args&&... args) { this->log(src_loc{}, level::critical, fmt, std::forward<_Args>(args)...); }

    }; // class
} // namespace