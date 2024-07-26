#pragma once
#include <stdint.h>
#include <optional>
#include <string_view>
#include <charconv>

#include "math3d.hh"

namespace core::ebrcv24g_proto
{
	/**
	 * Ref: EBRCV24GV5_rev11.pdf,7p
	 *
	 * Output data packet format (ASCII):
	 *   <channel_id>-<sensor_id>,<data_1>,<data_2>,...,<data_N>,<CRLF>
	 *
	 * Output data packet format (HEX):
	 *    +-----+----+----+-----+-----+-----+-----+-----+
	 *    | SOP | CH | ID | D_1 | D_2 | ... | D_N | CHK |
	 *    +-----+----+----+-----+-----+-----+-----+-----+
	 *    - CH, ID fields are 1-byte, all other items are 2-bytes
	 *    - SOP        -> 0x5555
	 *    - D_1 .. D_N -> Sensor data; each data type is int16_t
	 *    - CHK        -> Checksum; sum of all BYTES including SOP. (ignore overflow)
	 *
	 * Data output order:
	 *   +----------------+---------+-------------------------+---------+----------------+-------+---------+-----------+
	 *   | Orientation    | Gyro    | Accel                   | Magnet  | Distance       | Temp  | Battery | TimeStamp |
	 *   +----------------+---------+-------------------------+---------+----------------+-------+---------+-----------+
	 *   | Euler [R,P,Y]  | [X,Y,Z] | Raw Accel [X,Y,Z]       | [X,Y,Z] | Local [X,Y,Z]  | [deg] | [%]     | [ms]      |
	 *   | Quat [Z,Y,X,W] |         | Linear Local [X,Y,Z]    |         | Global [X,Y,Z] |       |         |           |
	 *   |                |         | Linear Global [X,Y,Z]   |         |                |       |         |           |
	 *   |                |         | Velocity Local [X,Y,Z]  |         |                |       |         |           |
	 *   |                |         | Velocity Global [X,Y,Z] |         |                |       |         |           |
	 *   +----------------+---------+-------------------------+---------+----------------+-------+---------+-----------+
	 *   NOTE: Each item of the above data can be individually turned ON/OFF via setting commands.
	 *         The number of output data items cannot exceed 15. (if exceeded, an <os> error is respond)
	 *
	 * Example:
	 *   "100-0,0.0982,-0.0038,-0.0001,0.9951,-0.003,0.003,0.999,91,46307"
	 */

	enum class output_data_type {
		//orientation_euler, // TODO: support later
		orientation_quat,
		gyroscope,
		acceleration,
		magnet,
		distance,
		temperature,
		battery,
		timestamp,
	};
	
	enum class output_data_field_type {
		//orientation_euler_r, // TODO: support later
		//orientation_euler_p, // TODO: support later
		//orientation_euler_y, // TODO: support later
		orientation_quat_x,
		orientation_quat_y,
		orientation_quat_z,
		orientation_quat_w,
		gyroscope_x,
		gyroscope_y,
		gyroscope_z,
		acceleration_x,
		acceleration_y,
		acceleration_z,
		magnet_x,
		magnet_y,
		magnet_z,
		distance_x,
		distance_y,
		distance_z,
		temperature,
		battery,
		timestamp,
	};

	static constexpr uint16_t kMaxChipTimestampInMillis{ 60000 };

	using device_rf_addr_t = uint32_t;

	constexpr device_rf_addr_t make_device_rf_addr(uint32_t rf_channel_id, uint8_t rf_sensor_id) noexcept {
		return static_cast<device_rf_addr_t>(rf_channel_id) * 100u + static_cast<device_rf_addr_t>(rf_sensor_id);
	}

	struct device_output_packet_t {
		uint8_t rf_channel_id; // range: 0...125
		uint8_t rf_sensor_id; // range: 0...99
		Eigen::Quaterniond orientation;
		Eigen::Vector3d raw_acceleration;
		uint8_t battery_level; // range: 0...100
		uint16_t chip_timestamp; // range: 0...kMaxChipTimestampInMillis [ms]

		device_output_packet_t()
			: rf_channel_id{}
			, rf_sensor_id{}
			, orientation{}
			, raw_acceleration{}
			, battery_level{}
			, chip_timestamp{}
		{ }

		device_rf_addr_t get_device_rf_addr() const noexcept {
			return make_device_rf_addr(rf_channel_id, rf_sensor_id);
		}
	};

	namespace {
		namespace detail
		{
			template <typename _Ty>
			inline bool parse_numeric(
				const std::string_view token_sv, 
				_Ty& result,
				const bool parse_strict = false)
			{
				const char* const pbegin{ token_sv.data() };
				const char* const pend{ pbegin + token_sv.size() };
				const std::from_chars_result res{ std::from_chars(pbegin, pend, result) };
				if (res.ec != std::errc{}) { return false; }
				if (parse_strict && res.ptr != pend) { return false; }
				return true;
			}

		} // namespace detail
	}

	static inline bool try_parse_device_output_packet(
		const std::string_view packet_sv,
		device_output_packet_t& result)
	{
		std::string_view sv = packet_sv;
		if (const size_t end_pos{ sv.rfind("\r\n") };
			end_pos != std::string_view::npos) {
			sv = sv.substr(0, end_pos);
		}

		size_t output_data_order_idx{};
		for (size_t curr_pos{}, curr_tok_idx{}; curr_pos < sv.size(); ++curr_tok_idx)
		{
			size_t next_pos = sv.find(',', curr_pos);
			if (next_pos == std::string_view::npos) { next_pos = sv.size(); }

			const std::string_view curr_tok{ sv.substr(curr_pos, next_pos - curr_pos) };

			switch (curr_tok_idx) {
			// Channel ID & Sensor ID
			case 0: {
				const size_t split_pos{ curr_tok.find('-') };
				if (split_pos == std::string_view::npos) { return false; }
				if (!detail::parse_numeric(curr_tok.substr(0, split_pos), result.rf_channel_id, true)) { return false; }
				if (!detail::parse_numeric(curr_tok.substr(split_pos + 1), result.rf_sensor_id, true)) { return false; }
				break;
			}
			// Orientation; [Z,Y,X,W]
			case 1: {
				if (!detail::parse_numeric(curr_tok, result.orientation.z(), true)) { return false; }
				break;
			}
			case 2: {
				if (!detail::parse_numeric(curr_tok, result.orientation.y(), true)) { return false; }
				break;
			}
			case 3: {
				if (!detail::parse_numeric(curr_tok, result.orientation.x(), true)) { return false; }
				break;
			}
			case 4: {
				if (!detail::parse_numeric(curr_tok, result.orientation.w(), true)) { return false; }
				break;
			}
			// Raw Accel; [X,Y,Z]
			case 5: {
				if (!detail::parse_numeric(curr_tok, result.raw_acceleration.x(), true)) { return false; }
				break;
			}
			case 6: {
				if (!detail::parse_numeric(curr_tok, result.raw_acceleration.y(), true)) { return false; }
				break;
			}
			case 7: {
				if (!detail::parse_numeric(curr_tok, result.raw_acceleration.z(), true)) { return false; }
				break;
			}
			// Battery Level; [%]
			case 8: {
				if (!detail::parse_numeric(curr_tok, result.battery_level, true)) { return false; }
				break;
			}
			// Timestamp; [ms]
			case 9: {
				if (!detail::parse_numeric(curr_tok, result.chip_timestamp, true)) { return false; }
				break;
			}
			default:
				return false;
			}

			curr_pos = next_pos + 1;
		}

		return true;
	}

} // namespace
