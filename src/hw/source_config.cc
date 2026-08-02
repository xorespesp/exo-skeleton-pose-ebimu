#include "source_config.hh"

#include <format>
#include <variant>

namespace hw
{
    namespace
    {
        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    } // namespace

    source_backend_t get_source_backend(const source_config_t& config)
    {
        return std::visit(overloaded{
            [](const k4a_device_config&) { return source_backend_t::k4a; },
            [](const vz_device_config&)  { return source_backend_t::vz; },
            [](const recording_config&)  { return source_backend_t::recording; },
        }, config);
    }

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
