#pragma once

#include <Eigen/Geometry>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace gui
{
    // scrolling_buffer stores these in a std::vector, so they must be copyable.
    template <typename _Ty>
    concept scrolling_value = std::copyable<_Ty>;

    // Fixed-capacity ring buffer; once full, the oldest value is at `offset`.
    template <scrolling_value _Ty>
    class scrolling_buffer
    {
    public:
        using value_type = _Ty;

    public:
        int32_t max_size;
        int32_t offset;
        std::vector<value_type> data;

    public:
        explicit scrolling_buffer(int32_t max_size_ = 2000)
            : max_size{ max_size_ }
            , offset{ 0 }
        {
            data.reserve(max_size);
        }

        void add_value(const value_type& new_value) {
            if (static_cast<int32_t>(data.size()) < max_size) {
                data.push_back(new_value);
            } else {
                data[offset] = new_value;
                offset = (offset + 1) % max_size;
            }
        }

        template <typename... _Args>
        void emplace_value(_Args&&... args) {
            this->add_value(value_type{ std::forward<_Args>(args)... });
        }

        void clear() {
            if (!data.empty()) {
                data.clear();
                offset = 0;
            }
        }
    };

    // Maps a plot value to its scalar type and a span over its contiguous channels.
    template <typename _Ty>
    struct plot_traits_t;

    // Eigen column vectors (Vector2/3/4 f/d, and any fixed Nx1).
    template <typename _Scalar, int _Rows>
    struct plot_traits_t<Eigen::Matrix<_Scalar, _Rows, 1>>
    {
        using scalar_type = _Scalar;
        static std::span<const scalar_type> channels(const Eigen::Matrix<_Scalar, _Rows, 1>& v)
        {
            return { v.data(), static_cast<std::size_t>(_Rows) };
        }
    };

    // Eigen quaternions (Quaternionf/d), stored as (x, y, z, w).
    template <typename _Scalar>
    struct plot_traits_t<Eigen::Quaternion<_Scalar>>
    {
        using scalar_type = _Scalar;
        static std::span<const scalar_type> channels(const Eigen::Quaternion<_Scalar>& q)
        {
            return { q.coeffs().data(), 4 };
        }
    };

    // A buffer's samples laid out for a strided line plot. `_Scalar` is the sample type (float/double).
    template <typename _Scalar>
    struct plot_buffer_view_t
    {
        const _Scalar* xs{ nullptr };  // time base, strided by `stride`
        std::span<const _Scalar> ys{}; // per-value channels; channel k plotted from ys.data() + k, strided by `stride`
        int count{ 0 };                // sample count
        int offset{ 0 };               // ring index of the oldest sample
        int stride{ 0 };               // bytes between samples
        _Scalar t_hi{ 0 };             // newest sample time
    };

    // `_N` ring buffers sharing one timeline. Sample times are rebased to the first sample
    // so they stay float-precise; a backward jump (e.g. a seek) restarts the history.
    template <scrolling_value _Ty, std::size_t _N>
    class plot_buffer
    {
    public:
        using value_type = _Ty;
        using scalar_type = typename plot_traits_t<_Ty>::scalar_type;
        using sample_type = std::pair<scalar_type, _Ty>; // (time since the first sample, value)

        // Set the current time; call once before each round of push().
        void advance(double t)
        {
            if (_last_t && t < *_last_t) { this->clear(); } // backward jump: reset the history
            if (!_last_t) { _origin = t; } // first sample after a reset: fix the rebase reference
            _last_t = t;
        }

        // Append `v` to buffer `i` at the current (rebased) time.
        void push(std::size_t i, const value_type& v)
        {
            _bufs[i].add_value({ static_cast<scalar_type>(*_last_t - _origin), v });
        }

        void clear()
        {
            for (auto& b : _bufs) { b.clear(); }
            _last_t.reset();
        }

        // Strided view of buffer `i` (empty until the first push).
        plot_buffer_view_t<scalar_type> view(std::size_t i) const
        {
            const auto& b = _bufs[i];
            const int count = static_cast<int>(b.data.size());
            if (count == 0) { return {}; }
            const int newest = (b.offset + count - 1) % count;
            return {
                &b.data[0].first,
                plot_traits_t<value_type>::channels(b.data[0].second),
                count,
                b.offset,
                static_cast<int>(sizeof(sample_type)),
                b.data[newest].first,
            };
        }

    private:
        std::array<scrolling_buffer<sample_type>, _N> _bufs;
        double _origin{ 0.0 };         // first-sample time (rebase reference)
        std::optional<double> _last_t; // previous time; empty until the first advance()
    };

} // namespace gui
