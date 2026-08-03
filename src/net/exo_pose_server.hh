#pragma once
#include "app_config.hh"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace net
{
    class exo_pose_pipeline;

    // Protocol correlation id (Message.request_id)
    using req_id_t = uint32_t;

    // request_id value reserved for messages the server sends on its own
    // (status/pose/ended notifications); clients must use a non-zero id.
    inline constexpr req_id_t kServerNotifyReqId{ 0 };

    // uWS based exo-pose server. (protocol: exo_pose_proto.fbs)
    class exo_pose_server final
    {
    public:
        explicit exo_pose_server(
            const app::app_config_t& config, 
            bool annotate_frames = false // draws detections onto the frames the pipeline hands out.
        );
        ~exo_pose_server();

        exo_pose_server(const exo_pose_server&) = delete;
        exo_pose_server& operator=(const exo_pose_server&) = delete;
        exo_pose_server(exo_pose_server&&) = delete;
        exo_pose_server& operator=(exo_pose_server&&) = delete;

        bool is_listening() const;
        bool start(); // binds the WebSocket listener + event loop
        void stop();  // tears down the listener (the pipeline + loop stay alive)

        // Advance the server one tick (non-blocking): service the listener when up, and
        // always pump the pose pipeline (pull detections, recompute, broadcast if up).
        // The pipeline runs even while stopped, so an external render loop drives this
        // every frame regardless of listener state.
        void poll();

        // Convenience blocking mode: start(), run the loop until it ends, then stop().
        int run();

        // The pose pipeline this server drives.
        // A GUI debugger reads and controls the source and estimator through this;
        // single-threaded with poll().
        exo_pose_pipeline& pipeline();
        const exo_pose_pipeline& pipeline() const;

    private:
        // Advance the pipeline one step: pull detections and recompute, then, while the listener
        // is up, broadcast a pose frame, a stream-ended notice, and a refreshed status as needed.
        void _pump_pipeline();

        // Protocol serializers (loop thread) -> FlatBuffers `Message` bytes.
        // Pass req_id to echo the triggering command's id back on a reply;
        // NOTE: 0 is reserved for messages the server sends on its own.
        std::string _serialize_pose_frame() const;
        std::string _serialize_server_status(req_id_t req_id = kServerNotifyReqId) const;
        std::string _serialize_source_stream_ended() const;
        std::string _serialize_ack(
            bool ok,
            std::string_view msg,
            req_id_t req_id = kServerNotifyReqId
        ) const;

    private:
        struct impl_t;
        std::unique_ptr<impl_t> _imp;
    };

} // namespace net
