#pragma once
#include <Windows.h>
#include <optional>
#include <string>
#include <string_view>
#include <stdexcept>

namespace utils::string
{
	namespace {
		namespace detail
		{
			template <
				// Recommended to use only `CP_UTF8` value;
				// The ANSI code pages can be different on different computers, or can be changed for a single computer, leading to data corruption.
				UINT _MBSCodePage
			>
			std::optional<std::wstring> mbs_to_ws(
				const std::string_view mbs) noexcept
			{
				std::wstring ws;

				if (!mbs.empty())
				{
					// MultiByteToWideChar does not null-terminate an output string if the input string length is explicitly specified without a terminating null character.
					// To null-terminate an output string for this function, the application should pass in -1 or explicitly count the terminating null character for the input string.

					const int ws_len = ::MultiByteToWideChar(
						_MBSCodePage,
						MB_ERR_INVALID_CHARS,
						mbs.data(),
						static_cast<int>(mbs.length()),
						nullptr,
						0
					);

					if (ws_len <= 0) {
						return std::nullopt;
					}

					ws.resize(ws_len);
					::MultiByteToWideChar(
						_MBSCodePage,
						MB_ERR_INVALID_CHARS,
						mbs.data(),
						static_cast<int>(mbs.length()),
						&ws[0],
						ws_len
					);
				}

				return ws;
			}

			template <
				// Recommended to use only `CP_UTF8` value;
				// The ANSI code pages can be different on different computers, or can be changed for a single computer, leading to data corruption.
				UINT _MBSCodePage
			>
			std::optional<std::string> ws_to_mbs(
				const std::wstring_view ws) noexcept
			{
				std::string mbs;

				if (!ws.empty())
				{
					// WideCharToMultiByte does not null-terminate an output string if the input string length is explicitly specified without a terminating null character.
					// To null-terminate an output string for this function, the application should pass in -1 or explicitly count the terminating null character for the input string.

					const int mbs_len = ::WideCharToMultiByte(
						_MBSCodePage,
						WC_ERR_INVALID_CHARS,
						ws.data(),
						static_cast<int>(ws.length()),
						nullptr,
						0,
						nullptr,
						nullptr
					);

					if (mbs_len <= 0) {
						return std::nullopt;
					}

					mbs.resize(mbs_len);
					::WideCharToMultiByte(
						_MBSCodePage,
						WC_ERR_INVALID_CHARS,
						ws.data(),
						static_cast<int>(ws.length()),
						&mbs[0],
						mbs_len,
						nullptr,
						nullptr
					);
				}

				return mbs;
			}

		} // namespace detail
	} // namespace

	static inline std::wstring utf8_to_utf16(std::string_view utf8str) {
		const auto cvtret = detail::mbs_to_ws<CP_UTF8>(utf8str);
		if (!cvtret) { throw std::runtime_error{ "utf8_to_utf16() failed" }; }
		return cvtret.value();
	}

	static inline std::wstring utf8_to_utf16(const std::string& utf8str) {
		const auto cvtret = detail::mbs_to_ws<CP_UTF8>(utf8str);
		if (!cvtret) { throw std::runtime_error{ "utf8_to_utf16() failed" }; }
		return cvtret.value();
	}

	static inline std::string utf16_to_utf8(std::wstring_view utf16str) {
		const auto cvtret = detail::ws_to_mbs<CP_UTF8>(utf16str);
		if (!cvtret) { throw std::runtime_error{ "utf16_to_utf8() failed" }; }
		return cvtret.value();
	}

	static inline std::string utf16_to_utf8(const std::wstring& utf16str) {
		const auto cvtret = detail::ws_to_mbs<CP_UTF8>(utf16str);
		if (!cvtret) { throw std::runtime_error{ "utf16_to_utf8() failed" }; }
		return cvtret.value();
	}

} // namespace