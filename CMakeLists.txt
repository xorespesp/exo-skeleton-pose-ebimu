cmake_minimum_required (VERSION 3.28 FATAL_ERROR)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(TARGET_NAME exo-skeleton-pose)
project(${TARGET_NAME} VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
include(GNUInstallDirs)

add_executable(${TARGET_NAME})
target_sources(${TARGET_NAME} PRIVATE
    "src/main.cc"
    "src/source_address.hh"
    "src/source_address.cc"
    "src/app_config.hh"
    "src/app_config.cc"
    "src/cli_options.hh"
    "src/cli_options.cc"

    # camera sensor abstraction
    "src/hw/clock_anchor.hh"
    "src/hw/timestamp.hh"
    "src/hw/roi.hh"
    "src/hw/frame_format.hh"
    "src/hw/calibration.hh"
    "src/hw/source_backend.hh"
    "src/hw/source_config.hh"
    "src/hw/source_config.cc"
    "src/hw/sensor_frameset.hh"
    "src/hw/sensor_frame.hh"
    "src/hw/sensor_frame_source.hh"
    "src/hw/sensor_frame_observer.hh"
    "src/hw/sensor_frame_provider.hh"
    "src/hw/sensor_frame_provider.cc"

    # camera sensor backend implementation (Orbbec K4A Wrapper, temporary)
    "src/hw/backends/k4a_frame_source.hh"
    "src/hw/backends/k4a_frame_source.cc"

    # recording container (MCAP), and playback as a frame source
    "src/io/mcap_impl.cc"
    "src/io/mcap_record_player.hh"
    "src/io/mcap_record_player.cc"
    "src/io/recording_message.hh"
    "src/io/recording_message.cc"
    "src/io/recording_writer.hh"
    "src/io/recording_writer.cc"
    "src/io/recording_reader.hh"
    "src/io/recording_reader.cc"
    "src/io/frame_recorder.hh"
    "src/io/frame_recorder.cc"
    "src/io/calibration_io.hh"
    "src/io/calibration_io.cc"

    # dsp
    "src/dsp/one_euro_filter.hh"

    # tag detection / pose
    "src/pose/tag_detector.hh"
    "src/pose/tag_detector.cc"
    "src/pose/leg_ik.hh"
    "src/pose/hinge_angle.hh"
    "src/pose/joints_def.hh"
    "src/pose/view_plane.hh"
    "src/pose/pose_estimator_base.hh"
    "src/pose/frontal_pose_estimator.hh"
    "src/pose/frontal_pose_estimator.cc"
    "src/pose/sagittal_pose_estimator.hh"
    "src/pose/sagittal_pose_estimator.cc"

    # gui
    "src/gui/app_base.hh"
    "src/gui/app_renderer_sdl3.hh"
    "src/gui/app_renderer_sdl3.cc"
    "src/gui/frame_texture.hh"
    "src/gui/frame_texture.cc"
    "src/gui/log_console.hh"
    "src/gui/log_console.cc"
    "src/gui/pose_trace_recorder.hh"
    "src/gui/pose_trace_recorder.cc"
    "src/gui/plot_buffer.hh"
    "src/gui/debugger_app.hh"
    "src/gui/debugger_app.cc"

    # server (websocket + flatbuffers protocol)
    "src/net/exo_pose_pipeline.hh"
    "src/net/exo_pose_pipeline.cc"
    "src/net/exo_pose_server.hh"
    "src/net/exo_pose_server.cc"
)

target_include_directories(${TARGET_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

if (WIN32)
    target_compile_definitions(${TARGET_NAME} PRIVATE 
        NOMINMAX 
        WIN32_LEAN_AND_MEAN)
endif()

include(FetchCLI11)
include(FetchEigen)
include(FetchSpdlog)
include(FetchAprilTag)
include(FetchOrbbecK4A)
include(FetchSDL3)
include(FetchFlatBuffers)
include(FetchUWebSockets)
include(FetchNlohmannJson)

add_subdirectory(deps)

find_package(OpenCV REQUIRED)
find_package(unofficial-mcap CONFIG REQUIRED) # header-only; target unofficial::mcap::mcap

# Generate protocol bindings from the FlatBuffers schema.
set(FBS_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/proto/exo_pose_proto.fbs")

# C++ header -> build tree, compiled into the server.
set(FBS_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(FBS_GEN_HEADER "${FBS_GEN_DIR}/exo_pose_proto_generated.h")
add_custom_command(
    OUTPUT ${FBS_GEN_HEADER}
    COMMAND flatc --cpp -o ${FBS_GEN_DIR} ${FBS_SCHEMA}
    DEPENDS ${FBS_SCHEMA} flatc
    COMMENT "Generating FlatBuffers C++ header from exo_pose_proto.fbs"
    VERBATIM
)
target_sources(${TARGET_NAME} PRIVATE ${FBS_GEN_HEADER})
target_include_directories(${TARGET_NAME} PRIVATE ${FBS_GEN_DIR})

# TS bindings -> webview build tree
set(FBS_TS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/webview/src/generated")
set(FBS_TS_BARREL "${FBS_TS_DIR}/exo/proto.ts")
add_custom_command(
    OUTPUT ${FBS_TS_BARREL}
    COMMAND flatc --ts -o ${FBS_TS_DIR} ${FBS_SCHEMA}
    DEPENDS ${FBS_SCHEMA} flatc
    COMMENT "Generating FlatBuffers TS bindings for the webview"
    VERBATIM
)
add_custom_target(webview_protocol DEPENDS ${FBS_TS_BARREL})
add_dependencies(${TARGET_NAME} webview_protocol)

# Bindings for the vendored Foxglove schemas (recording message payloads).
# Two artifacts per schema:
#   - a C++ header, to build and read the message payloads
#   - the binary schema (.bfbs) baked into a byte array, to be written into the MCAP
#     Schema record. That record is what lets external tools decode a recording.
set(FG_FBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto/foxglove")
set(FG_MESSAGE_SCHEMAS CompressedImage RawImage CameraCalibration)

set(FG_FBS_FILES "${FG_FBS_DIR}/Time.fbs")            # included by every message schema
set(FG_GEN_HEADERS "${FBS_GEN_DIR}/Time_generated.h")
foreach (schema IN LISTS FG_MESSAGE_SCHEMAS)
    list(APPEND FG_FBS_FILES "${FG_FBS_DIR}/${schema}.fbs")
    list(APPEND FG_GEN_HEADERS "${FBS_GEN_DIR}/${schema}_generated.h")
endforeach()

add_custom_command(
    OUTPUT ${FG_GEN_HEADERS}
    COMMAND flatc --cpp -o ${FBS_GEN_DIR} ${FG_FBS_FILES}
    DEPENDS ${FG_FBS_FILES} flatc
    COMMENT "Generating FlatBuffers C++ headers from the Foxglove schemas"
    VERBATIM
)
target_sources(${TARGET_NAME} PRIVATE ${FG_GEN_HEADERS})

foreach (schema IN LISTS FG_MESSAGE_SCHEMAS)
    add_custom_command(
        OUTPUT "${FBS_GEN_DIR}/${schema}_bfbs.h"
        COMMAND flatc --schema --binary -o ${FBS_GEN_DIR} "${FG_FBS_DIR}/${schema}.fbs"
        COMMAND ${CMAKE_COMMAND}
                -DINPUT=${FBS_GEN_DIR}/${schema}.bfbs
                -DOUTPUT=${FBS_GEN_DIR}/${schema}_bfbs.h
                -DSYMBOL=k${schema}Bfbs
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedBinary.cmake"
        DEPENDS "${FG_FBS_DIR}/${schema}.fbs" "${FG_FBS_DIR}/Time.fbs" flatc
                "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedBinary.cmake"
        COMMENT "Embedding binary FlatBuffers schema ${schema}.bfbs"
        VERBATIM
    )
    target_sources(${TARGET_NAME} PRIVATE "${FBS_GEN_DIR}/${schema}_bfbs.h")
endforeach()

target_link_libraries(${TARGET_NAME} PRIVATE
    Eigen3::Eigen
    spdlog::spdlog_header_only
    apriltag::apriltag
    CLI11::CLI11
    k4a::k4a
    unofficial::mcap::mcap
    ${OpenCV_LIBS}
    imgui::imgui
    implot::implot
    implot3d::implot3d
    imfilebrowser::imfilebrowser
    uwebsockets::uwebsockets
    flatbuffers::flatbuffers
    nlohmann_json::nlohmann_json
)

setup_orbbec_k4a_binaries(${TARGET_NAME} INSTALL_PATH ${CMAKE_INSTALL_BINDIR})

########################################################################################
# Offline converter: K4A .mkv -> our .mcap
#
# The one tool that still reads the K4A recording format, so k4a::k4arecord stays linked
# here and out of the app. Reuses the recording writer and the K4A color/calibration path.
########################################################################################

set(K4AMKV2MCAP_NAME exo-k4amkv2mcap)
add_executable(${K4AMKV2MCAP_NAME}
    "tools/k4amkv2mcap/k4amkv2mcap.cc"
    "src/hw/backends/k4a_frame_source.cc"
    "src/io/mcap_impl.cc"
    "src/io/recording_message.cc"
    "src/io/recording_writer.cc"
    ${FG_GEN_HEADERS}
)
foreach (schema IN LISTS FG_MESSAGE_SCHEMAS)
    target_sources(${K4AMKV2MCAP_NAME} PRIVATE "${FBS_GEN_DIR}/${schema}_bfbs.h")
endforeach()
target_include_directories(${K4AMKV2MCAP_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src" ${FBS_GEN_DIR})
if (WIN32)
    target_compile_definitions(${K4AMKV2MCAP_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
target_link_libraries(${K4AMKV2MCAP_NAME} PRIVATE
    Eigen3::Eigen
    spdlog::spdlog_header_only
    CLI11::CLI11
    k4a::k4a
    k4a::k4arecord
    unofficial::mcap::mcap
    ${OpenCV_LIBS}
    flatbuffers::flatbuffers
)
setup_orbbec_k4a_binaries(${K4AMKV2MCAP_NAME} INSTALL_PATH ${CMAKE_INSTALL_BINDIR})

########################################################################################
# Vieworks VZ camera, driven through the Galaxy C++ API
#
# The SDK-facing half lives in one static library. Its include path, import library and
# warning suppression are properties of that target, so whatever links it needs to know
# nothing about where the SDK sits.
#
# Built only where the SDK is installed; its location comes from the VZ_WIN64_SDK_PATH
# environment variable the installer sets. Everything else builds without it.
########################################################################################

set(VZ_BACKEND_NAME exo-vz)
set(VZCAM_TEST_NAME exo-vzcam-test)
set(VZ_SDK_ROOT "$ENV{VZ_WIN64_SDK_PATH}")
set(VZ_SDK_AVAILABLE FALSE)
if (WIN32 AND VZ_SDK_ROOT AND EXISTS "${VZ_SDK_ROOT}/Development/inc/GalaxyIncludes.h")
    set(VZ_SDK_AVAILABLE TRUE)
endif()

if (VZ_SDK_AVAILABLE)
    message(STATUS "VZSolutionSDK found: ${VZ_SDK_ROOT}")

    add_library(${VZ_BACKEND_NAME} STATIC
        "src/hw/backends/vz_device.hh"
        "src/hw/backends/vz_device.cc"
        "src/hw/backends/vz_frame_source.hh"
        "src/hw/backends/vz_frame_source.cc"
    )
    target_include_directories(${VZ_BACKEND_NAME}
        PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/src"
        PRIVATE "${VZ_SDK_ROOT}/Development/inc"
    )
    target_compile_definitions(${VZ_BACKEND_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    # The SDK headers carry non-UTF-8 bytes in their comments, which floods C4828 on a
    # UTF-8 build. Nothing here reads those comments, so silence it where they are included.
    target_compile_options(${VZ_BACKEND_NAME} PRIVATE /wd4828)
    target_link_libraries(${VZ_BACKEND_NAME}
        PUBLIC  ${OpenCV_LIBS} # cv::Mat crosses the interface
                Eigen3::Eigen  # so does calibration_t
        PRIVATE spdlog::spdlog_header_only
                "${VZ_SDK_ROOT}/Development/lib/x64/GxIAPICPPEx.lib"
    )

    # EXO_HAS_VZ_BACKEND is what lets the app build without the SDK: without it the provider
    # compiles a branch that reports a VZ source as unavailable instead of opening one.
    target_link_libraries(${TARGET_NAME} PRIVATE ${VZ_BACKEND_NAME})
    target_compile_definitions(${TARGET_NAME} PRIVATE EXO_HAS_VZ_BACKEND)

    ####################################################################################
    # VZ camera test harness
    #
    # Bring-up tool for the camera: enumerate, stream, walk the GenICam nodes. Reuses the
    # GUI shell (app_base / app_renderer_sdl3 / frame_texture / log_console) so only the
    # camera panel is its own.
    ####################################################################################
    add_executable(${VZCAM_TEST_NAME}
        "tools/vzcam-test/vzcam_test_app.cc"

        # reused GUI shell
        "src/gui/app_base.hh"
        "src/gui/app_renderer_sdl3.hh"
        "src/gui/app_renderer_sdl3.cc"
        "src/gui/frame_texture.hh"
        "src/gui/frame_texture.cc"
        "src/gui/log_console.hh"
        "src/gui/log_console.cc"

        # tag detection, run without intrinsics for the 2D overlay
        "src/pose/tag_detector.hh"
        "src/pose/tag_detector.cc"
    )
    target_include_directories(${VZCAM_TEST_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(${VZCAM_TEST_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(${VZCAM_TEST_NAME} PRIVATE
        ${VZ_BACKEND_NAME}
        Eigen3::Eigen
        spdlog::spdlog_header_only
        apriltag::apriltag
        ${OpenCV_LIBS}
        imgui::imgui
        implot::implot
        implot3d::implot3d
    )
else()
    message(STATUS "VZSolutionSDK not found (VZ_WIN64_SDK_PATH unset or invalid); skipping ${VZ_BACKEND_NAME} and ${VZCAM_TEST_NAME}")
endif()

########################################################################################
# Install
########################################################################################

if(DEFINED CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX
        "${CMAKE_CURRENT_SOURCE_DIR}/install"
        CACHE PATH "Install prefix" FORCE
    )
endif()
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

# Install the executables
install(TARGETS ${TARGET_NAME} ${K4AMKV2MCAP_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Anything reaching the VZ camera is only meaningful on a machine that has the SDK installed:
# its Galaxy runtime DLLs are found through the PATH entries the SDK installer adds, and the
# GenICam runtime additionally needs that installation's environment (GenTL producers, XML
# cache). Bundling the DLLs alone would not make it run elsewhere, so only executables are
# installed and the Galaxy runtime is left to the SDK (see PRE_EXCLUDE_REGEXES below).
if (VZ_SDK_AVAILABLE)
    install(TARGETS ${VZCAM_TEST_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# Install PDB next to the exe on MSVC for Debug / RelWithDebInfo builds
if(MSVC)
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}>
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        CONFIGURATIONS Debug RelWithDebInfo
        OPTIONAL
    )
endif()

# Bundle runtime DLLs that the exe depends on (no-op for fully-static deps).
# (Requires CMake 3.21+)
install(TARGETS ${TARGET_NAME}
    RUNTIME_DEPENDENCY_SET ${TARGET_NAME}_runtime_deps
)
install(RUNTIME_DEPENDENCY_SET ${TARGET_NAME}_runtime_deps
    # Matched against the dependency name lowercased, so these patterns must be lowercase too.
    PRE_EXCLUDE_REGEXES
        [[api-ms-]] [[ext-ms-]]            # Windows API set DLLs
        [[^gxiapi]]                        # Galaxy runtime, left to the SDK installation
    POST_EXCLUDE_REGEXES
        [[.*[\\/]system32[\\/].*\.dll]]    # Windows system DLLs
        [[^/(lib|usr/lib|usr/lib64)/.*]]   # Linux system libs
    DIRECTORIES $<TARGET_FILE_DIR:${TARGET_NAME}>
)

# Project docs / metadata
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/docs/"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
