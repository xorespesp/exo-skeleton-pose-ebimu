#include "exo_pose_server.hh"

#include "exo_pose_pipeline.hh"

#include "exo_pose_proto_generated.h"

#include <App.h>         // uWS
#include <libusockets.h> // us_create_timer / us_timer_set / us_timer_ext / us_timer_close
#include <uv.h>          // uv_loop_t / uv_run

#include <spdlog/spdlog.h>

#include <chrono>
#include <format>
#include <functional>
#include <optional>
#include <stdexcept>
#include <bit>

namespace net
{
    namespace fb = flatbuffers;
    namespace fb_proto = exo::proto;

    namespace
    {
        // windows.h (pulled in by uWebSockets) defines a GetMessage macro that shadows
        // the generated flatbuffers GetMessage() accessor; isolate the workaround here.
        const fb_proto::Message* get_fb_proto_root_msg(const uint8_t* data)
        {
#pragma push_macro("GetMessage")
#undef GetMessage
            return fb_proto::GetMessage(data);
#pragma pop_macro("GetMessage")
        }

        // Owns the libuv loop (server lifetime) and the per-listen uWS App + timer.
        // The loop is bound to uWS once and reused across start/stop; the App and timer are
        // rebuilt on each start_listening(). Teardown closes every socket and drains the loop
        // before the App is destroyed, so no poll callback can fire against a freed context.
        class uws_event_loop
        {
        public:
            uws_event_loop() {
                if (::uv_loop_init(&_uv_loop) != 0) {
                    throw std::runtime_error{ "uv_loop_init failed" };
                }
            }

            ~uws_event_loop() {
                this->stop_listening();
                this->_drain();
                ::uv_loop_close(&_uv_loop);
            }

            uws_event_loop(const uws_event_loop&) = delete;
            uws_event_loop& operator=(const uws_event_loop&) = delete;

            // Advance the loop one non-blocking tick.
            void tick() { ::uv_run(&_uv_loop, UV_RUN_NOWAIT); }

            // Block on the loop until it runs out of work. (the listen socket + timer keep it alive)
            void run_blocking() { ::uv_run(&_uv_loop, UV_RUN_DEFAULT); }

            bool is_listening() const { return _is_listening; }

            // Bring the listener up: build the App, let `on_configure` register routes, start a
            // periodic `on_tick`, and bind the listen socket. All-or-nothing; a failed listen
            // rolls the App + timer back. `on_tick` fires every `tick_interval` and drives
            // progress while blocked in run_blocking().
            bool start_listening(
                uint16_t port,
                const std::function<void(uWS::App&)>& on_configure,
                std::function<void()> on_tick,
                std::chrono::milliseconds tick_interval)
            {
                if (_is_listening) { return true; }

                // Seed uWS's thread-local loop with ours before the first App grabs it via
                // Loop::get(). Idempotent: later calls return the cached loop and ignore the arg.
                uWS::Loop::get(&_uv_loop);

                _on_tick = std::move(on_tick);
                _uws_app.emplace();
                on_configure(_uws_app.value());

                struct timer_context_t { uws_event_loop* self; };
                _us_tick_timer = ::us_create_timer(std::bit_cast<us_loop_t*>(uWS::Loop::get()), 0, sizeof(timer_context_t));
                std::bit_cast<timer_context_t*>(::us_timer_ext(_us_tick_timer))->self = this;
                const int interval_ms = static_cast<int>(tick_interval.count());
                ::us_timer_set(_us_tick_timer, [](us_timer_t* t) {
                    std::bit_cast<timer_context_t*>(::us_timer_ext(t))->self->_on_tick();
                }, interval_ms, interval_ms);

                bool listening = false;
                _uws_app->listen(port, [&listening](us_listen_socket_t* sock) {
                    listening = (sock != nullptr); // uWS invokes this synchronously during listen()
                });

                if (!listening) {
                    this->stop_listening(); // roll the half-built listener back
                    return false;
                }

                _is_listening = true;
                return true;
            }

            // Tear the listener down in the order uWS requires: gate publishing off, close every
            // socket, drain the loop so their poll + close callbacks complete while the App is
            // still alive, then destroy the App (now free of open sockets).
            void stop_listening() {
                if (!_uws_app) { return; }

                _is_listening = false;
                if (_us_tick_timer) {
                    ::us_timer_close(_us_tick_timer);
                    _us_tick_timer = nullptr;
                }
                _uws_app->close();
                this->_drain();
                _uws_app.reset();
                _on_tick = nullptr;
            }

            // Publish to all subscribers of `topic`; no-op unless the listener is up.
            bool publish(std::string_view topic, std::string_view msg) {
                if (!_is_listening) { return false; }
                return _uws_app->publish(topic, msg, uWS::OpCode::BINARY);
            }

        private:
            // Pump pending close callbacks to completion. (bounded)
            void _drain() {
                for (int i = 0; i < 8 && ::uv_run(&_uv_loop, UV_RUN_NOWAIT) != 0; ++i) {}
            }

        private:
            uv_loop_t _uv_loop{};
            bool _is_listening{ false };
            std::optional<uWS::App> _uws_app;      // listen socket + ws/http contexts, while listening
            us_timer_t* _us_tick_timer{ nullptr }; // periodic pipeline tick, while listening
            std::function<void()> _on_tick;
        };

        struct uws_socket_userdata_t
        {
            bool accepted{ false };   // false for a rejected (over-capacity) socket
            bool handshaked{ false }; // set once a Hello carrying our protocol version arrives
        };

        // WebSocket close codes used on rejection.
        constexpr int kCloseTryAgainLater = 1013; // another client already holds the single slot
        constexpr int kCloseProtocolError = 1002; // the client is not speaking this protocol

    } // namespace

    // --- implementation --------------------------------------------------------------
    struct exo_pose_server::impl_t
    {
        app::app_config_t config; // the installation this server serves

        uws_event_loop uws_loop;    // uWS loop + listener
        exo_pose_pipeline pipeline; // source + detection + estimator
        size_t client_count{ 0 };   // connected clients; source released when it hits 0

        impl_t(
            const app::app_config_t& cfg, 
            bool annotate)
            : config{ cfg }
            , pipeline{ annotate }
        { }
    };

    // --- exo_pose_server -------------------------------------------------------------
    exo_pose_server::exo_pose_server(
        const app::app_config_t& config, 
        bool annotate_frames)
        : _imp{ std::make_unique<impl_t>(config, annotate_frames) }
    { }

    exo_pose_server::~exo_pose_server() {
        this->stop();
    }

    bool exo_pose_server::is_listening() const { return _imp->uws_loop.is_listening(); }

    exo_pose_pipeline& exo_pose_server::pipeline() { return _imp->pipeline; }
    const exo_pose_pipeline& exo_pose_server::pipeline() const { return _imp->pipeline; }

    app::app_config_t& exo_pose_server::config() { return _imp->config; }
    const app::app_config_t& exo_pose_server::config() const { return _imp->config; }

    bool exo_pose_server::start()
    {
        if (_imp->uws_loop.is_listening()) { return true; }

        // Register the WebSocket route + command handlers. The handlers outlive start()
        // (owned by the App) and the server outlives the loop, so capturing `this` is safe.
        const auto on_configure = [this](uWS::App& app)
        {
            app.ws<uws_socket_userdata_t>("/*", {
                .compression = uWS::DISABLED,
                .open = [this](auto* ws) {
                    const std::string peer{ ws->getRemoteAddressAsText() };
                    if (_imp->client_count >= 1) {
                        spdlog::warn("server: rejecting client {}: another client already holds the single slot", peer);
                        ws->end(kCloseTryAgainLater, "another client is already connected");
                        return;
                    }
                    ws->getUserData()->accepted = true;
                    ++_imp->client_count;
                    spdlog::info("server: client {} connected, waiting handshake..", peer);
                },
                .message = [this](auto* ws, std::string_view msg, uWS::OpCode /*op*/) {
                    if (!ws->getUserData()->accepted) {
                        spdlog::debug("server: rx {} bytes on a rejected socket, ignored", msg.size());
                        return;
                    }

                    // Drop the client: log the reason and put it on the websocket close frame
                    const auto reject_client = [ws](int close_code, std::string_view reason) {
                        spdlog::warn("server: rejecting client {}: {}", ws->getRemoteAddressAsText(), reason);
                        ws->end(close_code, reason);
                    };

                    const auto* data = std::bit_cast<const uint8_t*>(msg.data());
                    fb::Verifier verifier{ data, msg.size() };
                    if (!fb_proto::VerifyMessageBuffer(verifier))
                    {
                        reject_client(kCloseProtocolError, std::format("malformed message ({} bytes)", msg.size()));
                        return;
                    }

                    const fb_proto::Message* m = get_fb_proto_root_msg(data);
                    const req_id_t req = m->request_id(); // echoed back on the reply for correlation

                    spdlog::debug("server: rx {} (request id {}, {} bytes)",
                        fb_proto::EnumNamePayload(m->payload_type()), req, msg.size());

                    if (const bool is_valid_req_id = req != kServerNotifyReqId;
                        !is_valid_req_id)
                    {
                        // 0 is reserved for server notify events,
                        // so a client command carrying request id 0 is a protocol violation. reject it.
                        reject_client(kCloseProtocolError, "request_id must be non-zero");
                        return;
                    }

                    // Handshake: Check protocol version
                    if (m->payload_type() == fb_proto::Payload_Hello)
                    {
                        const auto client_ver = m->payload_as_Hello()->proto_version();
                        const auto server_ver = static_cast<uint32_t>(fb_proto::ProtocolVersion_Current);
                        if (client_ver != server_ver) {
                            reject_client(kCloseProtocolError, std::format("protocol version mismatch: server {}, client {}", server_ver, client_ver));
                            return;
                        }

                        ws->getUserData()->handshaked = true;
                        spdlog::info("server: client handshake ok (proto version: {})", server_ver);

                        // Both peers now agree on the schema, so payloads are safe to send:
                        // subscribe to the broadcast topics and hand over the current status.
                        ws->subscribe("pose");
                        ws->subscribe("status");
                        ws->send(this->_serialize_ack(true, "handshake ok", req), uWS::OpCode::BINARY);
                        spdlog::debug("server: tx ServerStatus (post-handshake)");
                        ws->send(this->_serialize_server_status(), uWS::OpCode::BINARY);
                        return;
                    }

                    if (!ws->getUserData()->handshaked)
                    {
                        reject_client(kCloseProtocolError, "handshake required: Hello must be the first message");
                        return;
                    }

                    /////////////////////////////////////////////////////////////////////////////////////
                    // NOTE: Past this point, the client is known to speak our protocol version
                    /////////////////////////////////////////////////////////////////////////////////////

                    // Commands reach into the pipeline, the only code here that can throw; a failing
                    // one must never take down the loop, so it ends in an error Ack.
                    try
                    {
                        // Pure query(no Ack, no broadcast):
                        // reply with the current status to this client only.
                        if (m->payload_type() == fb_proto::Payload_GetServerStatus)
                        {
                            spdlog::debug("server: tx ServerStatus (reply to request id {})", req);
                            ws->send(this->_serialize_server_status(req), uWS::OpCode::BINARY);
                            return;
                        }

                        // A source/rest command marks the pipeline's status changed; poll()
                        // rebroadcasts the ServerStatus to all clients on the next tick.
                        std::string ack;

                        switch (m->payload_type())
                        {
                            case fb_proto::Payload_StartPoseStream:
                            {
                                // Re-opening would rebuild the provider and drop a captured rest pose.
                                if (_imp->pipeline.is_source_open()) {
                                    ack = this->_serialize_ack(true, "already streaming", req);
                                    break;
                                }
                                // What to open is the installation's business, not the client's.
                                const bool ok = _imp->pipeline.open_source(_imp->config);
                                ack = this->_serialize_ack(ok,
                                    !_imp->config.camera.source.has_value()
                                        ? "the server is configured with no source"
                                        : (ok ? "streaming" : "the configured source failed to open"),
                                    req);
                                break;
                            }
                            case fb_proto::Payload_StopPoseStream:
                                _imp->pipeline.close_source();
                                ack = this->_serialize_ack(true, "stopped", req);
                                break;
                            case fb_proto::Payload_CalibrateRestPose:
                            {
                                const bool ok = _imp->pipeline.calibrate_rest_pose();
                                ack = this->_serialize_ack(ok, ok ? "rest pose calibrated" : "no computable joint rotation", req);
                                break;
                            }
                            case fb_proto::Payload_ClearRestPose:
                                _imp->pipeline.clear_rest_pose();
                                ack = this->_serialize_ack(true, "rest pose cleared", req);
                                break;
                            default:
                                spdlog::warn("server: unsupported message: {}", fb_proto::EnumNamePayload(m->payload_type()));
                                ack = this->_serialize_ack(false, "unsupported message", req);
                                break;
                        }

                        ws->send(ack, uWS::OpCode::BINARY);
                    }
                    catch (const std::exception& e)
                    {
                        spdlog::error("server: command handler error: {}", e.what());
                        ws->send(this->_serialize_ack(false, "internal error", req), uWS::OpCode::BINARY);
                    }
                },
                .close = [this](auto* ws, int code, std::string_view message) {
                    if (!ws->getUserData()->accepted) { return; } // rejected socket was never counted
                    --_imp->client_count;
                    spdlog::info("server: client disconnected (code {}{}{})",
                        code, message.empty() ? "" : ": ", message);
                    // Release the source once the last client leaves so a monitor GUI reflects
                    // it and a live device is freed.
                    if (_imp->client_count == 0)
                    {
                        _imp->pipeline.close_source();
                        spdlog::info("server: last client disconnected; source released");
                    }
                }
            });
        };

        // ~120 Hz pipeline tick; poll() also drives it, and the shared frame seq makes
        // whichever runs second a no-op.
        const auto on_tick = [this]() {
            this->_pump_pipeline();
        };

        using namespace std::chrono_literals;
        const bool ok = _imp->uws_loop.start_listening(
            _imp->config.server.port,
            on_configure,
            on_tick,
            8ms // ~120 Hz pipeline tick
        );

        if (ok) {
            spdlog::info("server: listening on ws://localhost:{} (protocol version {})",
                _imp->config.server.port, static_cast<uint32_t>(fb_proto::ProtocolVersion_Current));
        } else {
            spdlog::error("server: failed to listen on port {} (already in use?)", _imp->config.server.port);
        }
        return ok;
    }

    void exo_pose_server::stop()
    {
        if (_imp->uws_loop.is_listening())
        {
            spdlog::info("server: stopping ({} client(s) will be dropped)", _imp->client_count);
        }
        _imp->uws_loop.stop_listening();
        _imp->client_count = 0;
    }

    void exo_pose_server::poll()
    {
        // Service the listener when up (its timer pumps the pipeline inside uv_run), then pump
        // directly so the pipeline still advances while stopped. The shared frame seq makes the
        // direct call a no-op when the timer already consumed the frame.
        if (_imp->uws_loop.is_listening()) {
            _imp->uws_loop.tick();
        }
        this->_pump_pipeline();
    }

    int exo_pose_server::run()
    {
        if (!this->start()) {
            this->stop();
            return -1;
        }

        // The configured source comes up with the server, device or recording alike.
        if (_imp->config.camera.source.has_value()) {
            spdlog::info("server: auto-opening the configured source");
            _imp->pipeline.open_source(_imp->config);
        } else {
            // Said now rather than left for the first client to discover as a failed open.
            spdlog::warn("server: no source is configured; pose data cannot flow until one is");
        }

        // Blocks while the listen socket + timer keep the loop alive.
        // (the timer drives _pump_pipeline())
        _imp->uws_loop.run_blocking();
        this->stop();
        return 0;
    }

    void exo_pose_server::_pump_pipeline()
    {
        // poll() consumes the pipeline's per-step signals; broadcast each while the listener is up.
        // A status change (from a client command or a GUI action) is dropped while stopped, since
        // there are no subscribers; a client receives the current status once its handshake completes.
        const exo_pose_pipeline::poll_result_t r = _imp->pipeline.poll();
        if (_imp->uws_loop.is_listening())
        {
            // Pose frames go out at source rate, so they are logged at trace; the two rare
            // status broadcasts get a debug line each.
            if (r.new_pose)
            {
                const std::string frame = this->_serialize_pose_frame();
                spdlog::trace("server: tx PoseFrame #{} ({} bytes) to {} client(s)",
                    _imp->pipeline.current_frame_seq(), frame.size(), _imp->client_count);
                _imp->uws_loop.publish("pose", frame);
            }
            if (r.stream_end_reason.has_value())
            {
                const bool is_error = (r.stream_end_reason == hw::stream_end_reason_t::failed);
                spdlog::debug("server: tx SourceStreamEnded (error: {})", is_error);
                _imp->uws_loop.publish("status", this->_serialize_source_stream_ended(is_error));
            }
            if (r.status_changed)
            {
                spdlog::debug("server: tx ServerStatus (source open: {}, rest pose: {})",
                    _imp->pipeline.is_source_open(), _imp->pipeline.has_rest_pose());
                _imp->uws_loop.publish("status", this->_serialize_server_status());
            }
        }
    }

    std::string exo_pose_server::_serialize_pose_frame() const
    {
        fb::FlatBufferBuilder b;

        // Without an open source there is no estimator and every joint reports as lost.
        const pose::pose_estimator_base* est = _imp->pipeline.estimator();

        std::vector<fb::Offset<fb_proto::JointPose>> joints;
        joints.reserve(pose::kNumJoints);
        for (const auto& def : pose::get_joint_defs())
        {
            const pose::joint_state_t st = est ? est->get_joint_state(def.joint_id) : pose::joint_state_t{};

            const auto to_fb_quat = [](const Eigen::Quaterniond& q) {
                return fb_proto::Quat{ q.x(), q.y(), q.z(), q.w() };
            };

            // local_anim_rot (the IK animation rotation) is what drives the rig; absent when lost.
            // NOTE: fb_local_anim_rot must outlive CreateJointPose, which copies it immediately.
            fb_proto::Quat fb_local_anim_rot;
            if (st.local_anim_rot.has_value()) { fb_local_anim_rot = to_fb_quat(st.local_anim_rot.value()); }

            // The measured angles ride beside the rotation, each field in the convention its
            // schema comment states, read straight off the joint state.
            const auto to_fb_angle = [](const std::optional<double>& a) {
                return a.has_value() ? fb::Optional<double>{ a.value() } : fb::nullopt;
            };

            joints.push_back(fb_proto::CreateJointPose(
                b, static_cast<fb_proto::JointId>(def.joint_id),
                st.local_anim_rot.has_value() ? &fb_local_anim_rot : nullptr,
                to_fb_angle(st.sagittal_segment_angle),
                to_fb_angle(st.sagittal_clinical_angle),
                to_fb_angle(st.sagittal_included_angle)
            ));
        }

        const auto joints_vec = b.CreateVector(joints);
        const uint32_t frame_seq = _imp->pipeline.current_frame_seq();
        const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            _imp->pipeline.last_timestamp().time_since_epoch()).count();
        const auto pose_frame = fb_proto::CreatePoseFrame(b, frame_seq, timestamp_us, _imp->pipeline.has_rest_pose(), joints_vec);

        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_PoseFrame, pose_frame.Union(), kServerNotifyReqId));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string exo_pose_server::_serialize_server_status(req_id_t req_id) const
    {
        fb::FlatBufferBuilder b;

        const bool is_streaming = _imp->pipeline.is_source_open();

        const auto to_fb_string = [&b](std::string_view s) {
            return s.empty() ? b.CreateString("") : b.CreateString(s.data(), s.size());
        };

        const auto source_name_str = to_fb_string(
            is_streaming ? _imp->pipeline.source_name() : std::string{});
        const auto source_backend_str = to_fb_string(is_streaming
            ? hw::source_backend_to_str(_imp->pipeline.source_backend())
            : std::string_view{});

        int32_t w = 0, h = 0;
        if (is_streaming) {
            const auto res = _imp->pipeline.source_resolution();
            w = res.x(); h = res.y();
        }

        const auto status = fb_proto::CreateServerStatus(b, 
            is_streaming, 
            source_name_str, 
            source_backend_str, 
            w, h, 
            _imp->pipeline.has_rest_pose()
        );

        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_ServerStatus, status.Union(), req_id));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string exo_pose_server::_serialize_source_stream_ended(const bool is_error) const
    {
        fb::FlatBufferBuilder b;
        const char* msg = is_error ? "the source stream failed" : "the source reached its end";
        const auto ended = fb_proto::CreateSourceStreamEnded(b, is_error, b.CreateString(msg));
        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_SourceStreamEnded, ended.Union(), kServerNotifyReqId));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string exo_pose_server::_serialize_ack(
        bool ok,
        std::string_view msg,
        req_id_t req_id) const
    {
        if (ok) {
            spdlog::debug("server: tx Ack ok (request id {}): {}", req_id, msg);
        } else {
            spdlog::warn("server: tx Ack failed (request id {}): {}", req_id, msg);
        }

        fb::FlatBufferBuilder b;
        const auto ack = fb_proto::CreateAck(b, ok, b.CreateString(msg.data(), msg.size()));
        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_Ack, ack.Union(), req_id));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

} // namespace net
