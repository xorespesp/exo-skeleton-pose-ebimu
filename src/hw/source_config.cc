#include "source_config.hh"

#include <format>

namespace hw
{
    namespace
    {
        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    } // namespace

    std::string describe(const source_config_t& config)
    {
        return std::visit(overloaded{
            [](const k4a_device_config& c) {
                return std::format("k4a device #{}", c.device_index);
            },
            [](const vz_device_config& c) {
                return std::format("vz device '{}'", c.serial.empty() ? "<first>" : c.serial);
            },
            [](const recording_config& c) {
                return std::format("recording '{}'", c.file.filename().string());
            },
        }, config);
    }

} // namespace hw
