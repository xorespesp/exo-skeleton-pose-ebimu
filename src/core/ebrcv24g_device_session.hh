#pragma once
#include "io_context_pool.hh"
#include "ebrcv24g_proto.hh"

#include <boost/asio/serial_port.hpp>
#include <boost/pool/pool.hpp>
#include <boost/pool/pool_alloc.hpp>

#include "../utils/logger.hh"
#include "../utils/time_utils.hh"
#include "../utils/debug_utils.hh"

#include <functional>
#include <atomic>
#include <memory>
#include <array>
#include <deque>
#include <chrono>
#include <charconv>
#include <optional>

namespace core
{
	using namespace std::chrono_literals;

	/**
	 * EBIMU'24G'V52 - EB IMU(AHRS); 2.4GHz, F/W ver 5.20
	 * EBRCV'24G'V5  - EB Receiver;  2.4GHz, F/W ver 5.17
	 */

	class ebrcv24g_device_session;
	using ebrcv24g_device_session_ptr = std::shared_ptr<ebrcv24g_device_session>;

	class ebrcv24g_device_session
		: public std::enable_shared_from_this<ebrcv24g_device_session>
	{
	public:
		using this_type = ebrcv24g_device_session;
		using on_recv_callback_t = std::function<void(utils::clock::file_time, const ebrcv24g_proto::device_output_packet_t&)>;
		using on_close_callback_t = std::function<void(const boost::system::error_code&)>;

		static constexpr size_t kRxBuffSize{ 1024 };

		struct status_t final {
			std::atomic_bool flag_session_alive;
			boost::posix_time::time_duration keep_alive_timeout;

			status_t() { this->reset(); }

			void reset() {
				flag_session_alive = false;
				keep_alive_timeout = boost::posix_time::seconds{ 20 };
			}
		};

	private:
		boost::asio::io_context::strand _io_strand;
		boost::asio::serial_port _serial;
		boost::asio::deadline_timer _rx_deadline_timer;
		boost::asio::streambuf _rx_buff;

		status_t _session_stats;

		std::optional<utils::clock::file_time> _first_pck_ostime;
		std::optional<uint16_t> _prev_pck_chiptime;
		std::optional<std::chrono::nanoseconds> _curr_pck_reltime;

		std::unordered_map<
			ebrcv24g_proto::device_rf_addr_t,
			on_recv_callback_t
		> _rx_pck_handlers_map;

		on_close_callback_t _cb_close;

	public:
		ebrcv24g_device_session(
			boost::asio::io_context& io_ctx)
			: _io_strand{ io_ctx }
			, _serial{ io_ctx }
			, _rx_deadline_timer{ io_ctx }
		{
			LOG_TRACE("%s() ENTER", __func__);
		}

		virtual ~ebrcv24g_device_session()
		{
			LOG_TRACE("%s() ENTER", __func__);
		}
		
		void set_on_close(on_close_callback_t cb) {
			_cb_close = std::move(cb);
		}

		void set_keep_alive_timeout(const boost::posix_time::time_duration timeout) {
			_session_stats.keep_alive_timeout = timeout;
		}

		bool is_alive() const {
			return _serial.is_open() && _session_stats.flag_session_alive;
		}

		bool start(
			const std::string& device_port_name)
		{
			LOG_INFO("open port \"%s\" ..", device_port_name.c_str());
			boost::system::error_code ec;
			_serial.open(device_port_name, ec);
			if (ec) {
				LOG_ERROR("failed to open port! (ec = %d)", ec.value());
				return false;
			}

			_serial.set_option(boost::asio::serial_port_base::baud_rate{ 921600 });
			_serial.set_option(boost::asio::serial_port_base::character_size{ 8 });
			_serial.set_option(boost::asio::serial_port_base::parity{ boost::asio::serial_port_base::parity::none });
			_serial.set_option(boost::asio::serial_port_base::stop_bits{ boost::asio::serial_port_base::stop_bits::one });
			_serial.set_option(boost::asio::serial_port_base::flow_control{ boost::asio::serial_port_base::flow_control::none });

			LOG_INFO("open success!");
			_session_stats.flag_session_alive = true;

			this->init_device();
			return true;
		}

		void close() { this->close(boost::asio::error::operation_aborted); }
		void close(const boost::system::error_code& ec)
		{
			if (!_session_stats.flag_session_alive) { return; }

			auto self{ shared_from_this() };
			_io_strand.dispatch(
				[this, self, ec = ec]() {
					LOG_INFO("close start.. (ec = %d)", ec.value());

					_session_stats.reset();

					_rx_deadline_timer.cancel();

					{
						boost::system::error_code ec_;
						_serial.write_some(boost::asio::buffer("<stop>"), ec_);
					}

					_serial.cancel();
					_serial.close();

					_first_pck_ostime.reset();
					_prev_pck_chiptime.reset();
					_curr_pck_reltime.reset();
					_rx_pck_handlers_map.clear();

					if (_cb_close) {
						_cb_close(ec);
					}

					LOG_INFO("close done!");
				});
		}

		void register_output_packet_handler(
			ebrcv24g_proto::device_rf_addr_t dev_rf_addr,
			on_recv_callback_t cb)
		{
			const auto [it, success] = _rx_pck_handlers_map.insert({ dev_rf_addr, std::move(cb) });
			if (!success) {
				THROW_EXCEPTION("packet handler already exists");
			}
		}

		void unregister_output_packet_handler(
			ebrcv24g_proto::device_rf_addr_t dev_rf_addr)
		{
			_rx_pck_handlers_map.erase(dev_rf_addr);
		}

	private:
		void init_device()
		{
			auto self{ shared_from_this() };
			_io_strand.dispatch(
				[this, self]() {
					LOG_INFO("init device...");

					std::string resp;

					this->write_command_sync("<stop>", resp);

					/**
					 * TX: <cfg>>\n
					 * RX: <
					 * 	sb:8    // curr baudrate (default:8)
					 * 	sor:16  // curr fps 62.5 (1000/16 fps)
					 * 	soc:1   // curr output mode (1:ascii, 2:hex)
					 * 	sof:2   // curr output pose format [R,P,Y] / [Z,Y,X,W] (1:euler, 2:quat)
					 * 	sog:0   // curr output gyro [X,Y,Z] (0:off, 1:on)
					 * 	soa:0   // curr output accel/velocity [X,Y,Z] (0:off, 1:raw accel, 2:local accel, 3:global accel, 4:local velo, 5:global velo)
					 * 	som:0   // curr output magneto (0:off, 1:on)
					 * 	sod:2   // curr output distance (0:off, 1:local, 2:global)
					 * 	sot:0   // curr output temperature (0:off, 1:on)
					 * 	sob:1   // curr output battery level (0:off, 1:on)
					 * 	sots:1  // curr output timestamp (0:off, 1:on)
					 * 	sch:100 // curr rf channel id (default:100)
					 * 	mid:14  // curr max rf channel id (default:14)
					 * 	pons:0  // curr auto start mode (0:off, 1:on, default:1)
					 * 	><ok>
					 */
					this->write_command_sync("<cfg>>", resp);

					if (!(
						// <soc1> - set output mode (1:ascii, 2:hex)
						this->write_command_sync("<soc1>", resp) &&

						// <sof2> - set output pose format (1:euler, 2:quat)
						this->write_command_sync("<sof2>", resp) &&

						// <sog0> - set output gyro (0:off, 1:on)
						this->write_command_sync("<sog0>", resp) &&

						// <soa1> - set output accel/velocity (0:off, 1:raw accel, 2:local accel, 3:global accel, 4:local velo, 5:global velo)
						this->write_command_sync("<soa1>", resp) &&

						// <som0> - set output magneto (0:off, 1:on)
						this->write_command_sync("<som0>", resp) &&

						// <sod0> - set output distance (0:off, 1:local, 2:global)
						this->write_command_sync("<sod0>", resp) &&

						// <sob1> - set output battery level (0:off, 1:on)
						this->write_command_sync("<sob1>", resp) &&

						// <sots1> - set output timestamp (0:off, 1:on)
						this->write_command_sync("<sots1>", resp) &&

						// <??ssa4> - Set acceleration sensitivity (default:3)
						this->write_command_sync("<??ssa3>", resp) &&

						// <??lpfa9> - Set acceleration low-pass-filter cutoff freq (default:3; no-lpf:9)
						this->write_command_sync("<??lpfa3>", resp) &&

						// <??avca_e0> - Set acceleration active-vibration-cancellation (0:off, 1:on)
						this->write_command_sync("<??avca_e0>", resp)
						))
					{
						LOG_WARN("device initial setup did not complete successfully. output may be unexpected.");
					}

					LOG_INFO("start device...");
					this->write_command_sync("<start>", resp);

					LOG_INFO("start reading...");
					this->read_next_output_packets();
				});
		}

		bool write_command_sync(
			const std::string& cmd,
			std::string& resp/* out */,
			const std::chrono::milliseconds max_wait_timeout = 5ms)
		{
			resp.clear();

			if (!_session_stats.flag_session_alive) { return false; }

			boost::system::error_code ec;
			_serial.write_some(boost::asio::buffer(cmd), ec);
			if (ec) {
				LOG_ERROR("failed to send command \"%s\" (ec = %d)"
					, cmd.c_str()
					, ec.value()
				);
				return false;
			}

			boost::asio::streambuf resp_buff;
			const std::string resp_delim{ ">" };
			const size_t bytes_transferred{ boost::asio::read_until(_serial, resp_buff, resp_delim, ec) };
			if (ec) {
				LOG_ERROR("failed to recv command resp \"%s\" (ec = %d)"
					, cmd.c_str()
					, ec.value()
				);
				return false;
			}

			resp = std::string(
				boost::asio::buffers_begin(resp_buff.data()),
				boost::asio::buffers_begin(resp_buff.data()) + bytes_transferred
			);

			std::this_thread::sleep_for(200ms);

			// Consume through the first delimiter.
			//resp_buff.consume(bytes_transferred);

			return resp == "<ok>";
		}

	private:
		void read_next_output_packets()
		{
			if (!_session_stats.flag_session_alive) { return; }
			
			this->update_rx_deadline_timer();
			auto self{ shared_from_this() };

			static const std::string delim{ "\r\n" };
			_serial.async_read_some(_rx_buff.prepare(kRxBuffSize), boost::asio::bind_executor(_io_strand,
				[this, self](const boost::system::error_code& ec, const std::size_t bytes_transferred)
				{
					// Check whether the session was closed 
					// before this completion handler had a chance to run.
					if (!_session_stats.flag_session_alive) { return; }

					if (!ec)
					{
						const utils::clock::file_time curr_pck_ostime{ utils::clock::file_time::utcnow() };

						size_t num_of_rx_pcks{};

						_rx_buff.commit(bytes_transferred);
						std::string_view input_sv{ static_cast<const char*>(_rx_buff.data().data()), _rx_buff.size() };
						if (const size_t end_pos{ input_sv.rfind(delim.c_str()) };
							end_pos != std::string_view::npos)
						{
							const size_t bytes_consumed{ end_pos + delim.size() };
							input_sv = input_sv.substr(0, bytes_consumed);

							size_t curr_pos{};
							while (curr_pos < input_sv.size()) {
								size_t next_pos = input_sv.find(delim.c_str(), curr_pos);
								if (next_pos == std::string_view::npos) { next_pos = input_sv.size(); }

								const std::string_view packet_sv{ input_sv.substr(curr_pos, next_pos - curr_pos) };
								this->handle_output_packet(packet_sv, curr_pck_ostime);
								++num_of_rx_pcks;

								curr_pos = next_pos + delim.size();
							}

							_rx_buff.consume(bytes_consumed);
						}

						//LOG_TRACE("bytes_transferred = %lu, num_of_rx_pcks = %lu"
						//	, bytes_transferred
						//	, num_of_rx_pcks
						//);

						this->read_next_output_packets();
					}
					else if (ec != boost::asio::error::operation_aborted)
					{
						LOG_ERROR("serial read error %d", ec.value());
						this->close(ec);
					}
				}));
		}

		void handle_output_packet(
			const std::string_view packet_data_sv,
			const utils::clock::file_time packet_recv_ostime)
		{
			ebrcv24g_proto::device_output_packet_t parsed_pck{};
			if (ebrcv24g_proto::try_parse_device_output_packet(packet_data_sv, parsed_pck))
			{
				if (!_first_pck_ostime) {
					_first_pck_ostime = packet_recv_ostime;
				}

				const uint16_t curr_pck_chiptime = parsed_pck.chip_timestamp;
				const uint16_t prev_pck_chiptime = _prev_pck_chiptime.value_or(curr_pck_chiptime);
				const std::chrono::milliseconds chiptime_diff{ (prev_pck_chiptime > curr_pck_chiptime) // overflow?
					? static_cast<std::chrono::milliseconds::rep>((ebrcv24g_proto::kMaxChipTimestampInMillis - prev_pck_chiptime) + curr_pck_chiptime)
					: static_cast<std::chrono::milliseconds::rep>(curr_pck_chiptime - prev_pck_chiptime)
				};

				if (!_curr_pck_reltime) {
					_curr_pck_reltime = chiptime_diff;
				} else {
					*_curr_pck_reltime += chiptime_diff;
				}

				const utils::clock::file_time curr_pck_abstime{
					_first_pck_ostime->value + static_cast<uint64_t>(_curr_pck_reltime->count() / 100/* convert unit: 100-ns */)
				};

				// update prev_pck_chiptime
				_prev_pck_chiptime = curr_pck_chiptime;

				if (const auto it = _rx_pck_handlers_map.find(parsed_pck.get_device_rf_addr());
					it != _rx_pck_handlers_map.end())
				{
					const auto& [_, cb] = *it;
					if (cb) {
						cb(curr_pck_abstime, parsed_pck);
					}
				}
			}
			else
			{
				LOG_ERROR("output packet parse failed!");
			}
		}

		void update_rx_deadline_timer()
		{
			auto self{ shared_from_this() };
			_rx_deadline_timer.expires_from_now(_session_stats.keep_alive_timeout);
			_rx_deadline_timer.async_wait(boost::asio::bind_executor(_io_strand,
				[this, self](const boost::system::error_code& /*ec*/)
				{
					// Check whether the session was closed 
					// before this completion handler had a chance to run.
					if (!_session_stats.flag_session_alive) { return; }

					if (_rx_deadline_timer.expires_at() <= boost::asio::deadline_timer::traits_type::now())
					{
						LOG_ERROR("session deadline timeout");
						this->close();
					}
				}));
		}

	}; // class

} // namespace