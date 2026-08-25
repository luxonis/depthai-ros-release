cmake_minimum_required(VERSION 3.15)
project(FoxgloveWebSocket VERSION 0.0.1 LANGUAGES CXX)

find_package(nlohmann_json REQUIRED)
find_package(websocketpp REQUIRED)

# It uses headers that require symbols from openssl
find_package(OpenSSL REQUIRED)

add_library(foxglove_websocket STATIC src/base64.cpp src/parameter.cpp src/serialization.cpp src/server_factory.cpp)
add_library(foxglove-websocket::foxglove-websocket ALIAS foxglove_websocket)

# Separate SYSTEM include directories from other include directories
target_include_directories(foxglove_websocket
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(foxglove_websocket
    SYSTEM PUBLIC
        ${nlohmann_json_INCLUDE_DIRS}
        ${websocketpp_INCLUDE_DIRS}
)

target_link_libraries(foxglove_websocket
    PUBLIC
        nlohmann_json::nlohmann_json
        websocketpp::websocketpp
        OpenSSL::SSL
        OpenSSL::Crypto
)

set_target_properties(foxglove_websocket PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

if (MSVC)
  target_compile_options(foxglove_websocket PRIVATE /W4 /bigobj)
else()
  target_compile_options(foxglove_websocket PRIVATE -Wall -Wextra -Wpedantic -Wold-style-cast -Wfloat-equal)
endif()
if(FOXGLOVE_WEBSOCKET_INSTALL)
    # Installation
    include(GNUInstallDirs)

    install(TARGETS foxglove_websocket
        EXPORT foxglove_websocketTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # For Windows DLLs
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(FILES ${CMAKE_SOURCE_DIR}/LICENSE
        DESTINATION ${CMAKE_INSTALL_PREFIX}/licenses/
    )

    # Export the targets to a file
    install(EXPORT foxglove_websocketTargets
        FILE foxglove_websocketTargets.cmake
        NAMESPACE foxglove-websocket::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foxglove_websocket
    )

    # Create a version file
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/foxglove_websocketConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    # Config file
    configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/foxglove_websocketConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/foxglove_websocketConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foxglove_websocket
    )

    # Install the config files
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/foxglove_websocketConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/foxglove_websocketConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foxglove_websocket
    )

    # Export from the build tree
    export(EXPORT foxglove_websocketTargets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/foxglove_websocketTargets.cmake"
        NAMESPACE foxglove-websocket::
    )
    export(PACKAGE foxglove_websocket)
endif()
