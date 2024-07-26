#pragma once
#include "c_format.hh"
#include "logger.hh"

#include <Windows.h>
#include <exception>
#include <filesystem>
#include <type_traits>
#include <string>
#include <string_view>

#define ASSERT_ALWAYS(EXPR) ::utils::debug_assert(EXPR, #EXPR, __FILE__, __LINE__ )
#define THROW_EXCEPTION(C_FMT, ...) ::utils::debug_throw(__FILE__, __LINE__, C_FMT, __VA_ARGS__)

namespace utils
{
    enum class dbgrpt_mbox_type {
        error,
        warn,
        info
    };

    template <dbgrpt_mbox_type _MBoxType, typename... _Args>
    static void show_dbgrpt_mbox(
        const std::string& c_fmt,
        _Args&&... args)
    {
        /**
         * [Refs]
         * C:\Program Files (x86)\Windows Kits\10\Source\10.0.20348.0\ucrt\internal\winapi_thunks.cpp
         * C:\Program Files (x86)\Windows Kits\10\Source\10.0.20348.0\ucrt\misc\dbgrpt.cpp
         * C:\Program Files (x86)\Windows Kits\10\Source\10.0.20348.0\ucrt\misc\crtmbox.cpp
         */

        std::string mbox_title, mbox_body;
        uint32_t mbox_flags = MB_TASKMODAL | MB_SETFOREGROUND | MB_ABORTRETRYIGNORE;
        if constexpr (_MBoxType == dbgrpt_mbox_type::error) {
            mbox_title = "ERROR REPORT";
            mbox_flags |= MB_ICONERROR;
        }
        if constexpr (_MBoxType == dbgrpt_mbox_type::warn) {
            mbox_title = "WARNING REPORT";
            mbox_flags |= MB_ICONWARNING;
        }
        if constexpr (_MBoxType == dbgrpt_mbox_type::info) {
            mbox_title = "INFORMATION REPORT";
            mbox_flags |= MB_ICONINFORMATION;
        }

        mbox_body = utils::string::c_format(c_fmt, std::forward<_Args>(args)...);

        switch (::MessageBoxA(
            HWND_DESKTOP,
            mbox_body.c_str(),
            mbox_title.c_str(),
            mbox_flags
        )) {
        case IDABORT: { ::TerminateProcess(::GetCurrentProcess(), 3); break; }
        case IDRETRY: { ::__debugbreak(); break; }
        case IDIGNORE:
        default: { break; }
        }
    }

    static inline void debug_assert(
        const bool assert_expr,
        const std::string_view assert_expr_str,
        const std::string_view src_file_path,
        const uint32_t src_file_line)
    {
        if (!assert_expr)
        {
            const std::string_view src_file_name{ src_file_path.substr(src_file_path.find_last_of("/\\") + 1) }; 
            show_dbgrpt_mbox<dbgrpt_mbox_type::error>(
                "Runtime assertion failed!\n\n"
                "File: %.*s\n"
                "Line: %lu\n"
                "Expression: %.*s\n"
                "Thread: 0x%X\n\n"
                "(Press Retry to debug the application)"
                , static_cast<int>(src_file_name.length()), src_file_name.data()
                , src_file_line 
                , static_cast<int>(assert_expr_str.length()), assert_expr_str.data()
                , ::GetCurrentThreadId() 
            ); 
        }
    }

    template <typename... _Args>
    [[noreturn]]
    static inline void debug_throw(
        const std::filesystem::path& file,
        const uint32_t line,
        const std::string_view format,
        _Args&&... args)
    {
        std::runtime_error e{ "" };

#pragma warning(push)
#pragma warning(disable : 4996)
        int len = ::_snprintf(nullptr, 0, format.data(), args...);
        if (len > 0) {
            int buf_len = len + 1;
            char* buf = static_cast<char*>(::calloc(sizeof(char), buf_len));
            ::_snprintf(buf, buf_len, format.data(), std::forward<_Args>(args)...);
            e = std::runtime_error{ buf };
            ::free(buf);
        }
#pragma warning(pop)

        LOG_CRITICAL(L"Exception thrown at: <%S:%u> -> %S"
            , file.filename().string().c_str()
            , line
            , e.what()
        );

        throw e;
    }

} // namespace