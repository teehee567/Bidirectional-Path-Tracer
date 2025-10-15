cmake_minimum_required(VERSION 3.10)
project(RayTracer)
set(CMAKE_CXX_STANDARD 17)

find_package(PNG REQUIRED)

# Pick up all .cpp sources (add CONFIGURE_DEPENDS so it reconfigures when files are added)
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

# (Optional) collect headers just so they show up in IDEs
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS src/*.h)

add_executable(RayTracer
    ${SOURCES}
    ${HEADERS}
)

# Make "color.h", "ray.h", etc. resolvable without subfolder prefixes
target_include_directories(RayTracer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/image
    ${CMAKE_CURRENT_SOURCE_DIR}/src/image/external
    ${CMAKE_CURRENT_SOURCE_DIR}/src/materials
    ${CMAKE_CURRENT_SOURCE_DIR}/src/materials/textures
    ${CMAKE_CURRENT_SOURCE_DIR}/src/materials/volumes
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects/primatives
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects
    ${CMAKE_CURRENT_SOURCE_DIR}/src/acceleration
)

target_link_libraries(RayTracer PRIVATE PNG::PNG)