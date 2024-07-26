#pragma once
#include <type_traits>
#include <cstring>

namespace utils
{
    template <class _Dst, class _Src>
    static inline _Dst bit_cast(_Src src) noexcept
    {
        static_assert(sizeof(_Dst) == sizeof(_Src), "!!");
        static_assert(std::is_standard_layout_v<_Src> && std::is_trivial_v<_Src>, "!!");
        static_assert(std::is_standard_layout_v<_Dst> && std::is_trivial_v<_Dst>, "!!");

        _Dst dst;
        std::memcpy(std::addressof(dst), std::addressof(src), sizeof(_Dst));
        return dst;
    }

} // namespace