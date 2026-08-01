#pragma once
#include "sensor_frame.hh"

#include <memory>
#include <utility>

namespace hw
{
    // Single frameset capture from a backend.
    class sensor_frameset final
    {
    public:
        explicit sensor_frameset(std::shared_ptr<sensor_frame> color_frame)
            : _color_frame{ std::move(color_frame) }
        { }

        // Null when the capture carried no colour image.
        const std::shared_ptr<sensor_frame>& color_frame() const noexcept { return _color_frame; }

    private:
        std::shared_ptr<sensor_frame> _color_frame;
    };

} // namespace hw
