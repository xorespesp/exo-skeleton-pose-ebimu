#pragma once
#include "bit_cast.hh"

#include <Windows.h>
#include <time.h>
#include <chrono>

namespace utils::clock
{
	class file_time final
	{
	public:
		uint64_t value{}; // Unit: 100-ns

	public:
		constexpr file_time() noexcept = default;
		constexpr explicit file_time(uint64_t val/* Unit: 100-ns */) noexcept : value{val} {}
		constexpr file_time(FILETIME ft) noexcept : value{ (static_cast<uint64_t>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime } {}

		constexpr double to_nsec() const noexcept { return static_cast<double>(value) * 1e+02; }
		constexpr double to_usec() const noexcept { return static_cast<double>(value) * 1e-01; }
		constexpr double to_msec() const noexcept { return static_cast<double>(value) * 1e-04; }
		constexpr double to_sec() const noexcept { return static_cast<double>(value) * 1e-07; }

		inline SYSTEMTIME to_systime() const {
			const FILETIME ft{ static_cast<FILETIME>(*this) };
			SYSTEMTIME st{};
			if (!::FileTimeToSystemTime(&ft, &st)) {
				throw std::runtime_error{ "FileTimeToSystemTime failed" };
			}
			return st;
		}

		constexpr bool operator==(const file_time rhs) const noexcept { return value == rhs.value; }
		constexpr bool operator!=(const file_time rhs) const noexcept { return !(*this == rhs); }
		constexpr bool operator <(const file_time rhs) const noexcept { return value < rhs.value; }
		constexpr bool operator >(const file_time rhs) const noexcept { return value > rhs.value; }

		constexpr file_time& operator+=(const file_time rhs) noexcept { value += rhs.value; return *this; }
		constexpr file_time& operator-=(const file_time rhs) noexcept { value -= rhs.value; return *this; }
		friend constexpr file_time operator+(file_time lhs, const file_time rhs) noexcept { lhs += rhs; return lhs; }
		friend constexpr file_time operator-(file_time lhs, const file_time rhs) noexcept { lhs -= rhs; return lhs; }

		constexpr operator FILETIME() const noexcept {
			return FILETIME{ static_cast<DWORD>(value & 0xFFFFFFFF), static_cast<DWORD>(value >> 32) };
		}

#if defined(CPPWINRT_VERSION)
		inline operator winrt::file_time() const noexcept {
			return winrt::file_time{ value };
		}
#endif // ^^^ CPPWINRT_VERSION ^^^

	public:
		static inline file_time utcnow() noexcept
		{
			FILETIME ft_utc{};
			::GetSystemTimePreciseAsFileTime(&ft_utc);
			return file_time{ ft_utc };
		}

		static inline file_time now()
		{
			FILETIME ft_utc{};
			::GetSystemTimePreciseAsFileTime(&ft_utc);

			// NOTE: To account for summer time when converting a file time to a local time, 
			//       use the following sequence of functions in place of using FileTimeToLocalFileTime:
			//         1. FileTimeToSystemTime
			//         2. SystemTimeToTzSpecificLocalTime
			//         3. SystemTimeToFileTime

			FILETIME ft_local{};
			if (!::FileTimeToLocalFileTime(&ft_utc, &ft_local)) {
				throw std::runtime_error{ "FileTimeToLocalFileTime failed" };
			}

			return file_time{ ft_local };
		}

		static inline file_time from_systime(const SYSTEMTIME st)
		{
			FILETIME ft{};
			if (!::SystemTimeToFileTime(&st, &ft)) {
				throw std::runtime_error{ "SystemTimeToFileTime failed" };
			}

			return file_time{ ft };
		}

	}; // class

} // namespace