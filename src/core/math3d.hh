#pragma once
#include <type_traits>
#include <numeric>
#include <limits>
#include <cmath>

__pragma(warning(push)) // vvv Supress Eigen's internal warning C4127, C4819 vvv
__pragma(warning(disable:4127))
__pragma(warning(disable:4819))
// https://eigen.tuxfamily.org/index.php?title=Main_Page
// https://eigen.tuxfamily.org/dox-devel/group__QuickRefPage.html
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/EulerAngles>
__pragma(warning(pop))  // ^^^ Supress Eigen's internal warning C4127, C4819 ^^^

namespace core::math
{
	static constexpr double
		kPi = 3.141592653589793238462643383280,
		k2Pi = 6.283185307179586476925286766559,
		kPiDiv2 = 1.570796326794896619231321691640,
		kInvPi = 0.318309886183790671537767526745,
		kDegToRad = 0.017453292519943295769236907685,
		kRadToDeg = 57.29577951308232087679815481410;
	
	template <typename _Ty>
	constexpr _Ty rad2deg(const _Ty rad) noexcept {
		static_assert(std::is_floating_point_v<_Ty>, "!!");
		return rad * static_cast<_Ty>(kRadToDeg);
	}

	template <typename _Ty>
	constexpr _Ty deg2rad(const _Ty deg) noexcept {
		static_assert(std::is_floating_point_v<_Ty>, "!!");
		return deg * static_cast<_Ty>(kDegToRad);
	}

	static inline Eigen::Quaterniond quat_from_eulerZYX(const Eigen::Vector3d& euler_angles_zyx/* ZYX order; Unit: [degree] */)
	{
		const double
			euler_x_pitch_rad = deg2rad(euler_angles_zyx[2]/* x */),
			euler_y_yaw_rad = deg2rad(euler_angles_zyx[1]/* y */),
			euler_z_roll_rad = deg2rad(euler_angles_zyx[0]/* z */);

		const Eigen::AngleAxisd
			pitch_ang{ euler_x_pitch_rad, Eigen::Vector3d::UnitX() },
			yaw_ang{ euler_y_yaw_rad, Eigen::Vector3d::UnitY() },
			roll_ang{ euler_z_roll_rad, Eigen::Vector3d::UnitZ() };

		Eigen::Quaterniond q_rot;
		q_rot = roll_ang * yaw_ang * pitch_ang;
		return q_rot;
	}

	template <typename EigenEulerSystemType>
	static inline Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q_target)
	{
		return Eigen::Vector3d/* euler_angles_in_radian */{
			Eigen::EulerAngles<double, EigenEulerSystemType>(
				q_target.normalized().toRotationMatrix()
			).angles()
		};
	}

	static inline Eigen::Vector3d quat_to_eulerXYZ(const Eigen::Quaterniond& q_target) {
		return quat_to_euler<Eigen::EulerSystemXYZ>(q_target);
	}
	
	static inline Eigen::Vector3d quat_to_eulerZYX(const Eigen::Quaterniond& q_target) {
		return quat_to_euler<Eigen::EulerSystemZYX>(q_target);
	}

	static inline Eigen::Vector3d quat_to_eulerYXZ(const Eigen::Quaterniond& q_target) {
		return quat_to_euler<Eigen::EulerSystemYXZ>(q_target);
	}

	static inline Eigen::Quaterniond quat_combine(
		const Eigen::Quaterniond& q_first,
		const Eigen::Quaterniond& q_second)
	{
		return Eigen::Quaterniond{ q_second * q_first };
	}

	static inline Eigen::Quaterniond quat_relative(
		const Eigen::Quaterniond& q_target,
		const Eigen::Quaterniond& q_parent)
	{
		return Eigen::Quaterniond{ q_parent.inverse() * q_target };
	}

} // namespace