cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(PROJECT_NAME exo-skeleton-pose-ebimu)
project(${PROJECT_NAME} VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}
  "src/main.cc"
  "src/core/ebrcv24g_device_session.hh"
  "src/core/ebrcv24g_proto.hh"
  "src/core/io_context_pool.hh"
  "src/core/math3d.hh"
  "src/core/pose_estimator.hh"
  "src/utils/bit_cast.hh"
  "src/utils/codecvt.hh"
  "src/utils/c_format.hh"
  "src/utils/debug_utils.hh"
  "src/utils/logger.hh"
  "src/utils/mutex.hh"
  "src/utils/time_utils.hh"
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(magic_enum REQUIRED)

target_link_libraries(${PROJECT_NAME} PRIVATE 
	Boost::boost
	Eigen3::Eigen
	spdlog::spdlog_header_only
	magic_enum::magic_enum
)