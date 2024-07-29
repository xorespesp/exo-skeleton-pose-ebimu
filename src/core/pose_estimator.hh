#pragma once
#include "io_context_pool.hh"
#include "ebrcv24g_device_session.hh"

#include "../utils/logger.hh"
#include "../utils/debug_utils.hh"
#include "math3d.hh"

#include <boost/pool/pool.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/circular_buffer.hpp>

#include <type_traits>
#include <unordered_map>
#include <vector>
#include <list>

namespace core
{
    enum joint_id_t {
        JOINT_HIP_R, // root
        JOINT_HIP_L, // root
        JOINT_KNEE_R,
        JOINT_KNEE_L,
        JOINT_ANKLE_R,
        JOINT_ANKLE_L,
        NumOfJoints
    };

    constexpr joint_id_t kInvalidJointID{ NumOfJoints };

    static const std::vector<std::pair<joint_id_t/* child */, joint_id_t/* parent */>> kJointsHierarchyMap = {
        std::make_pair(JOINT_HIP_R, JOINT_HIP_R), // root
        std::make_pair(JOINT_HIP_L, JOINT_HIP_L), // root
        std::make_pair(JOINT_KNEE_R, JOINT_HIP_R),
        std::make_pair(JOINT_KNEE_L, JOINT_HIP_L),
        std::make_pair(JOINT_ANKLE_R, JOINT_KNEE_R),
        std::make_pair(JOINT_ANKLE_L, JOINT_KNEE_L)
    };

    static const std::unordered_map<joint_id_t, Eigen::Quaterniond> kJointsTPoseQuaternionMap = {
        { JOINT_HIP_R, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) },
        { JOINT_HIP_L, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) },
        { JOINT_KNEE_R, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) },
        { JOINT_KNEE_L, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) },
        { JOINT_ANKLE_R, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) },
        { JOINT_ANKLE_L, math::quat_from_eulerZYX(Eigen::Vector3d{ /* z */0.0, /* y */0.0, /* x */0.0 }) }
    };

    struct joint_data_t {
        joint_id_t joint_id;
        Eigen::Quaterniond quat_global;
        Eigen::Quaterniond quat_local; // relative to parent

        joint_data_t()
            : joint_id{ kInvalidJointID }
            , quat_global{ std::nan(""), std::nan(""), std::nan(""), std::nan("") }
            , quat_local{ std::nan(""), std::nan(""), std::nan(""), std::nan("") }
        {}

        bool is_valid() const noexcept {
            return joint_id != kInvalidJointID;
        }
    };

    class motion_frame final {
    private:
        std::array<joint_data_t, NumOfJoints> _joints_map;

    public:
        motion_frame() : _joints_map{} {}

        auto begin() const noexcept { return _joints_map.cbegin(); }
        auto end() const noexcept { return _joints_map.cend(); }

        auto begin() noexcept { return _joints_map.begin(); }
        auto end() noexcept { return _joints_map.end(); }

        const joint_data_t& get_joint_data(joint_id_t jid) const {
            return _joints_map.at(jid);
        }

        joint_data_t& get_joint_data(joint_id_t jid) {
            return _joints_map.at(jid);
        }

        size_t num_of_joints() const noexcept {
            return _joints_map.size();
        }

        void reset() {
            std::fill(_joints_map.begin(), _joints_map.end(), joint_data_t{});
        }
    };

    struct ebimu_device_status_t {
        ebrcv24g_proto::device_rf_addr_t device_rf_addr;
        joint_id_t mapped_joint_id;
        uint32_t battery_level;
        float rx_pps_value;
    };

    class pose_estimator final {
    public:
        struct options_t {
            std::string ebrcv_device_com_port;
            std::unordered_map<joint_id_t, ebrcv24g_proto::device_rf_addr_t> ebimu_device_joint_map;

            options_t() = default;
        };

    private:
        template <typename _Ty>
        using pooled_list = std::list<_Ty,
            boost::fast_pool_allocator<_Ty, boost::default_user_allocator_new_delete, boost::details::pool::default_mutex>
        >;

        template <typename _Ty>
        using pooled_vector = std::vector<_Ty,
            boost::pool_allocator<_Ty, boost::default_user_allocator_new_delete, boost::details::pool::default_mutex>
        >;

        class ebimu_device_context_t final
        {
        private:
            const ebrcv24g_proto::device_rf_addr_t _dev_rf_addr;
            const joint_id_t _mapped_joint_id;
            std::weak_ptr<ebrcv24g_device_session> _ebrcv_dev_session;

            boost::circular_buffer<std::pair<utils::clock::file_time, ebrcv24g_proto::device_output_packet_t>> _rx_pcks_q;
            utils::spin_lock _rx_pcks_q_lock;

            // for pps calculation
            boost::circular_buffer<utils::clock::file_time> _rx_times_q;
            utils::spin_lock _rx_times_q_lock;

            std::optional<Eigen::Quaterniond> _prev_sensor_orientation;
            Eigen::Quaterniond _curr_quat_global;
            std::atomic<uint8_t> _curr_batt_level;

        public:
            ebimu_device_context_t(
                ebrcv24g_device_session_ptr ebrcv_dev_session,
                ebrcv24g_proto::device_rf_addr_t dev_rf_addr,
                joint_id_t mapped_joint_id)
                : _dev_rf_addr{ dev_rf_addr }
                , _mapped_joint_id{ mapped_joint_id }
                , _ebrcv_dev_session{ ebrcv_dev_session }
                , _rx_pcks_q{ 60 * 10 }
                , _rx_pcks_q_lock{}
                , _rx_times_q{ 60 * 10 }
                , _rx_times_q_lock{}
                , _curr_quat_global{ kJointsTPoseQuaternionMap.at(mapped_joint_id) }
                , _curr_batt_level{ 0 }
            {
                ebrcv_dev_session->register_output_packet_handler(
                    dev_rf_addr,
                    std::bind(&ebimu_device_context_t::_handle_device_output_packet, this, std::placeholders::_1, std::placeholders::_2)
                );
            }

            ~ebimu_device_context_t() {
                auto ebrcv_dev_session_locked = _ebrcv_dev_session.lock();
                if (ebrcv_dev_session_locked) {
                    ebrcv_dev_session_locked->unregister_output_packet_handler(_dev_rf_addr);
                }
            }

            ebimu_device_context_t(const ebimu_device_context_t&) = delete;
            ebimu_device_context_t& operator= (const ebimu_device_context_t&) = delete;

            constexpr ebrcv24g_proto::device_rf_addr_t dev_rf_addr() const noexcept {
                return _dev_rf_addr;
            }

            constexpr joint_id_t mapped_joint_id() const noexcept {
                return _mapped_joint_id;
            }

            uint8_t current_battery_level() const noexcept {
                return _curr_batt_level.load();
            }

            float current_rx_pps() const noexcept {
                const auto tp_now = utils::clock::file_time::utcnow();

                std::scoped_lock lk{ _rx_times_q_lock };
                const size_t q_size = _rx_times_q.size();
                if (q_size) {
                    const float
                        first_pck_delta = static_cast<float>(tp_now.to_sec() - _rx_times_q.front().to_sec()),
                        last_pck_delta = static_cast<float>(tp_now.to_sec() - _rx_times_q.back().to_sec());
                    return (last_pck_delta <= 1.0f)
                        ? static_cast<float>(q_size) / first_pck_delta
                        : 0.0f;
                }

                return 0.0f;
            }

            bool fetch_rx_pck_nonbuffered(
                ebrcv24g_proto::device_output_packet_t& fetched_rx_pck/* out */)
            {
                std::scoped_lock lk{ _rx_pcks_q_lock };
                if (!_rx_pcks_q.empty()) {
                    const auto& [_, rx_pck] = _rx_pcks_q.back();
                    fetched_rx_pck = rx_pck;
                    return true;
                }

                return false;
            }

            bool fetch_rx_pck_buffered(
                ebrcv24g_proto::device_output_packet_t& fetched_rx_pck/* out */)
            {
                std::scoped_lock lk{ _rx_pcks_q_lock };
                if (!_rx_pcks_q.empty()) {
                    const auto& [_, rx_pck] = _rx_pcks_q.front();
                    fetched_rx_pck = rx_pck;
                    _rx_pcks_q.pop_front();
                    return true;
                }

                return false;
            }
            
            bool fetch_rx_pcks_buffered(
                std::vector<ebrcv24g_proto::device_output_packet_t>& fetched_rx_pcks/* out */)
            {
                fetched_rx_pcks.clear(); {
                    std::scoped_lock lk{ _rx_pcks_q_lock };
                    fetched_rx_pcks.reserve(_rx_pcks_q.size());
                    while (!_rx_pcks_q.empty()) {
                        const auto& [_, rx_pck] = _rx_pcks_q.front();
                        fetched_rx_pcks.emplace_back(rx_pck);
                        _rx_pcks_q.pop_front();
                    }
                }

                return !fetched_rx_pcks.empty();
            }

        private:
            void _handle_device_output_packet(
                const utils::clock::file_time recv_os_timestamp,
                const ebrcv24g_proto::device_output_packet_t& output_pck)
            {
                if (_prev_sensor_orientation)
                {
                    const Eigen::Quaterniond quat_delta = math::quat_relative(
                        _prev_sensor_orientation.value(),
                        output_pck.orientation
                    );

                    _curr_quat_global = math::quat_combine(
                        _curr_quat_global,
                        quat_delta
                    );
                }

                _prev_sensor_orientation = output_pck.orientation;
                _curr_batt_level = output_pck.battery_level;

                {
                    auto output_pck_copy = output_pck;
                    output_pck_copy.orientation = _curr_quat_global; // adjust orientation
                    std::scoped_lock lk{ _rx_pcks_q_lock };
                    _rx_pcks_q.push_back(std::make_pair(recv_os_timestamp, output_pck_copy));
                }

                {
                    std::scoped_lock lk{ _rx_times_q_lock };
                    _rx_times_q.push_back(recv_os_timestamp);
                }
            }
        };

        using ebimu_device_context_ptr = std::shared_ptr<ebimu_device_context_t>;

        struct context_t
            : public utils::spin_lock
        {
            ebrcv24g_device_session_ptr ebrcv_session;

            std::unordered_map<
                ebrcv24g_proto::device_rf_addr_t,
                ebimu_device_context_ptr
            > ebimu_ctx_map;

            context_t() = default;
        };

    private:
        io_context_pool _io_pool;

        mutable std::recursive_mutex _mtx;
        std::unique_ptr<context_t> _ctx;

    public:
        pose_estimator()
            : _io_pool{}
        {
            _io_pool.run_async();
        }

        ~pose_estimator() { _io_pool.stop(); }

        bool is_running() const {
            std::scoped_lock lk{ _mtx };
            return _ctx != nullptr;
        }

        void run_async(
            const options_t& opts)
        {
            std::scoped_lock lk{ _mtx };
            ASSERT_ALWAYS(!this->is_running());

            auto new_ctx = std::make_unique<context_t>();

            new_ctx->ebrcv_session = std::make_shared<ebrcv24g_device_session>(_io_pool.get_io_context());
            new_ctx->ebrcv_session->set_on_close(
                [this](const boost::system::error_code& /*ec*/) -> void
                {
                    this->stop();
                });

            for (const auto [target_joint_id, device_rf_addr] : opts.ebimu_device_joint_map) {
                new_ctx->ebimu_ctx_map[device_rf_addr] = std::make_shared<ebimu_device_context_t>(
                    new_ctx->ebrcv_session,
                    device_rf_addr,
                    target_joint_id
                );
            }

            if (!new_ctx->ebrcv_session->start(opts.ebrcv_device_com_port)) {
                LOG_ERROR("Failed to start ebrcv session");
                return;
            }

            _ctx = std::move(new_ctx);
            LOG_INFO("running...");
        }

        void stop()
        {
            std::scoped_lock lk{ _mtx };
            if (this->is_running())
            {
                LOG_INFO("stopping...");

                LOG_INFO("closing session...");
                if (_ctx->ebrcv_session) {
                    _ctx->ebrcv_session->close();
                }

                _ctx.reset();
                LOG_INFO("stopped.");
            }
        }

        bool try_get_motion_frame(
            motion_frame& mframe/* out */)
        {
            std::scoped_lock lk{ _mtx };
            if (!this->is_running()) {
                return false;
            }

            mframe.reset();

            size_t fetch_count = 0;
            for (const auto& [_, dev_ctx] : _ctx->ebimu_ctx_map)
            {
                ebrcv24g_proto::device_output_packet_t rx_pck;
                if (dev_ctx->fetch_rx_pck_nonbuffered(rx_pck)) {
                    const auto mapped_jid = dev_ctx->mapped_joint_id();
                    auto& new_jdata = mframe.get_joint_data(mapped_jid);
                    new_jdata.joint_id = mapped_jid;
                    new_jdata.quat_global = rx_pck.orientation;
                    ++fetch_count;
                }
            }

            if (fetch_count < 2) {
                return false;
            }

            return this->calculate_motion_frame_local_quaternions(mframe);
        }

        std::vector<ebimu_device_status_t> get_current_ebimu_device_stats() const
        {
            std::vector<ebimu_device_status_t> dev_stats;
            dev_stats.reserve(_ctx->ebimu_ctx_map.size());
            for (const auto& [dev_rf_addr, dev_ctx] : _ctx->ebimu_ctx_map)
            {
                ebimu_device_status_t new_stat{};
                new_stat.device_rf_addr = dev_rf_addr;
                new_stat.mapped_joint_id = dev_ctx->mapped_joint_id();
                new_stat.battery_level = dev_ctx->current_battery_level();
                new_stat.rx_pps_value = dev_ctx->current_rx_pps();
                dev_stats.emplace_back(new_stat);
            }

            return dev_stats;
        }

    private:

        bool calculate_motion_frame_local_quaternions(
            motion_frame& mframe)
        {
            size_t calculation_count = 0;

            for (const auto [target_jid, parent_jid] : kJointsHierarchyMap)
            {
                const bool is_root_joint = target_jid == parent_jid;
                const auto& parent_jdata = mframe.get_joint_data(parent_jid);
                auto& target_jdata = mframe.get_joint_data(target_jid);

                if (parent_jdata.is_valid() && target_jdata.is_valid())
                {
                    const Eigen::Quaterniond q_parent{
                        (is_root_joint)
                        ? Eigen::Quaterniond::Identity()
                        : parent_jdata.quat_global
                    };

                    const Eigen::Quaterniond q_target{ target_jdata.quat_global };
                    const Eigen::Quaterniond q_local{ math::quat_relative(q_target, q_parent) };

                    target_jdata.quat_local = q_local;
                    ++calculation_count;
                }
            }

            return calculation_count > 0;
        }

    }; // class
} // namespace