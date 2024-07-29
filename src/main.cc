#include "core/pose_estimator.hh"

#include <magic_enum.hpp>

#include <iostream>
#include <signal.h>
#include <conio.h>

void run_offscreen(const core::pose_estimator::options_t& opts)
{
    {
        std::string ebimu_device_joint_map_dump;
        for (const auto& [mapped_jid, dev_rf_addr] : opts.ebimu_device_joint_map) {
            const auto mapped_jname = magic_enum::enum_name(mapped_jid);
            ebimu_device_joint_map_dump += utils::string::c_format(
                "\n    Dev #%lu : %.*s"
                , dev_rf_addr
                , static_cast<int>(mapped_jname.size())
                , mapped_jname.data()
            );
        }
        LOG_INFO("ebimu device joint map:%s", ebimu_device_joint_map_dump.c_str());
    }

    core::pose_estimator estimator;
    estimator.run_async(opts);

    static bool s_flag_exit = false;
    ::signal(SIGINT, +[](int) { s_flag_exit = true; });

    LOG_INFO("waiting motion frames... (press ^C to interrupt)");
    while (!s_flag_exit)
    {
        core::motion_frame mframe;
        if (estimator.try_get_motion_frame(mframe))
        {
            std::string mframe_dump;
            for (const auto& jdata : mframe) {
                if (!jdata.is_valid()) { continue; }
                const auto jname = magic_enum::enum_name(jdata.joint_id);
                const auto euler_yxz_local = core::math::quat_to_eulerYXZ(jdata.quat_local) * core::math::kRadToDeg;
                mframe_dump += utils::string::c_format(
                    "\n    - %.*s ang(euler_yxz) = [%.2f, %.2f, %.2f]"
                    , static_cast<int>(jname.size())
                    , jname.data()
                    , euler_yxz_local.x()
                    , euler_yxz_local.y()
                    , euler_yxz_local.z()
                );
            }

            LOG_INFO("got motion frame: %s"
                , mframe_dump.c_str()
            );
        }
    }

    LOG_INFO("user interrupt detected. exiting...");
    estimator.stop();
}

int main(int argc, char** argv)
{
    int retval = -1;

    ::SetConsoleOutputCP(CP_UTF8);
    utils::logger::instance().init(utils::logger::init_option()
        .set_logger_name("exo-skeleton-pose-ebimu")
        .set_logger_level(utils::logger::level::debug)
        .enable_stdout_logging()
        //.enable_async_mode()
    );

    try
    {
        core::pose_estimator::options_t opts;
        opts.ebrcv_device_com_port = (argc > 1) ? argv[1] : "COM5";
        opts.ebimu_device_joint_map = {
            { core::JOINT_HIP_R, core::ebrcv24g_proto::make_device_rf_addr(100, 0) },
            { core::JOINT_KNEE_R, core::ebrcv24g_proto::make_device_rf_addr(100, 1) },
            { core::JOINT_ANKLE_R, core::ebrcv24g_proto::make_device_rf_addr(100, 2) },
            { core::JOINT_HIP_L, core::ebrcv24g_proto::make_device_rf_addr(100, 3) },
            { core::JOINT_KNEE_L, core::ebrcv24g_proto::make_device_rf_addr(100, 4) },
            { core::JOINT_ANKLE_L, core::ebrcv24g_proto::make_device_rf_addr(100, 5) }
        };

        run_offscreen(opts);

        LOG_INFO("all done.");
        retval = 0;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("main exception occurred -> %s", e.what());
    }
    catch (...)
    {
        LOG_CRITICAL("main unknown exception occurred");
#if defined(_DEBUG)
        ::puts("\nPress any key to debug application ...");
        static_cast<void>(::_getch());
        std::exception_ptr eptr = std::current_exception();
        ::__debugbreak();
        std::rethrow_exception(eptr);
#endif // ^^^ _DEBUG ^^^
    }

    utils::logger::instance().deinit();

    ::puts("\n\nPress any key to exit..");
    static_cast<void>(_getch());

    return retval;
}