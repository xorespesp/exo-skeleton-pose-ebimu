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
        explicit sensor_frameset(std::shared_ptr<sensor_frame> frame)
            : _frame{ std::move(frame) }
        { }

        // Null when the capture carried no image.
        const std::shared_ptr<sensor_frame>& frame() const noexcept { return _frame; }

    private:
        std::shared_ptr<sensor_frame> _frame;
    };

} // namespace hw
