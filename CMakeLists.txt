# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if ("${PROJECT_NAME}" STREQUAL "${COMPONENT_NAME}")
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(MeshtasticEspCompact)
else()
    add_subdirectory(components/EspMeshtasticCompact)
    idf_component_register()
endif()
