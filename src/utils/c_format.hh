#pragma once
#include <stdio.h>
#include <string>
#include <string_view>

namespace utils::string
{
	template <typename... _Args>
	static inline std::string c_format(
		const std::string& c_fmt,
		_Args&&... args)
	{
		std::string s;

		const int sz = ::_scprintf(c_fmt.c_str(), std::forward<_Args>(args)...);
		if (sz > 0) {
			s.resize(sz + 1);
			::_snprintf_s(s.data(), s.size(), s.size(), c_fmt.c_str(), std::forward<_Args>(args)...);
			s.pop_back();
		}

		return s;
	}

	template <typename... _Args>
	static inline std::wstring c_wformat(
		const std::wstring& c_wfmt,
		_Args&&... args)
	{
		std::wstring s;

		const int sz = ::_scwprintf(c_wfmt.c_str(), std::forward<_Args>(args)...);
		if (sz > 0) {
			s.resize(sz + 1);
			::_snwprintf_s(s.data(), s.size(), s.size(), c_wfmt.c_str(), std::forward<_Args>(args)...);
			s.pop_back();
		}

		return s;
	}

} // namespace
