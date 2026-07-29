if (NOT TARGET nlohmann_json::nlohmann_json)
    message(STATUS "Fetching nlohmann_json...")

    set(JSON_BuildTests OFF CACHE INTERNAL "")
    set(JSON_Install OFF CACHE INTERNAL "")

    FetchContent_Declare(
        nlohmann_json
        # Release tarball ships the CMake project + single-include header; far leaner
        # than a full git clone of the repo (which carries the large test corpus).
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    FetchContent_MakeAvailable(nlohmann_json)
endif()
