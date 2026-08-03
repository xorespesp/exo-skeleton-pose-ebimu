// Ported from: https://github.com/casiez/OneEuroFilter
#pragma once

#include <cmath>
#include <numbers>

namespace dsp
{
    struct one_euro_params_t
    {
        double min_cutoff_hz = 1.0; // lower min_cutoff -> smoother at rest;
        double beta = 0.05; // higher beta -> less lag in motion;
        double dcutoff_hz = 1.0; // dcutoff shapes the internal speed estimate.
    };

    // Smoothing factor for an exponential low-pass at `cutoff_hz`, sampled over
    // `dt_sec`: te = dt, tau = 1/(2*pi*cutoff), alpha = 1 / (1 + tau/te).
    // Callers must pass cutoff_hz > 0 and dt_sec > 0.
    [[nodiscard]] constexpr double alpha(double cutoff_hz, double dt_sec) noexcept
    {
        const double tau = 1.0 / (2.0 * std::numbers::pi * cutoff_hz);
        return 1.0 / (1.0 + tau / dt_sec);
    }

    // Exponential low-pass filter with a per-sample smoothing factor.
    // The first sample seeds the state (returned unchanged).
    class LowPassFilter
    {
        double _s = 0.0; // last filtered value
        double _y = 0.0; // last raw value
        bool _init = false;

    public:
        // s = a*value + (1-a)*s (seeds s = value on the first call).
        double filter(double value, double a) noexcept
        {
            const double result = _init ? (a * value + (1.0 - a) * _s) : value;
            _y = value;
            _s = result;
            _init = true;
            return result;
        }

        [[nodiscard]] bool has_last() const noexcept { return _init; }
        [[nodiscard]] double last_raw() const noexcept { return _y; }
        [[nodiscard]] double last_filtered() const noexcept { return _s; }

        void reset() noexcept { _s = 0.0; _y = 0.0; _init = false; }
    };

    // Scalar One Euro filter. `dt_sec` is the elapsed time since the previous
    // sample (the reference implementation derives it from timestamps; here it
    // is passed in so the caller owns the clock).
    class OneEuroFilter
    {
        double _min_cutoff; // min cutoff [Hz]; lower removes more jitter
        double _beta;       // speed coefficient; higher reduces lag in motion
        double _dcutoff;    // cutoff [Hz] for the derivative low-pass
        LowPassFilter _x;   // value low-pass
        LowPassFilter _dx;  // derivative low-pass

    public:
        OneEuroFilter(double min_cutoff = 1.0, double beta = 0.0, double dcutoff = 1.0) noexcept
            : _min_cutoff{ min_cutoff }, _beta{ beta }, _dcutoff{ dcutoff }
        { }

        [[nodiscard]] double filter(double value, double dt_sec) noexcept
        {
            const double dvalue = _x.has_last() ? (value - _x.last_filtered()) / dt_sec : 0.0;
            const double edvalue = _dx.filter(dvalue, alpha(_dcutoff, dt_sec));
            const double cutoff = _min_cutoff + _beta * std::abs(edvalue);
            return _x.filter(value, alpha(cutoff, dt_sec));
        }

        void reset() noexcept { _x.reset(); _dx.reset(); }

        void set_min_cutoff(double v) noexcept { _min_cutoff = v; }
        void set_beta(double v) noexcept { _beta = v; }
        void set_derivate_cutoff(double v) noexcept { _dcutoff = v; }
    };

} // namespace dsp
