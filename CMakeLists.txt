cmake_minimum_required(VERSION 3.10)
project(RayTracer)

set(CMAKE_CXX_STANDARD 20)

find_package(PNG REQUIRED)

add_executable(RayTracer
    src/main.cpp
)

set(RAYTRACER_INCLUDE_DIRS
    src
    src/acceleration
    src/core
    src/image
    src/materials
    src/materials/textures
    src/materials/volumes
    src/objects
    src/objects/primitives
)

target_include_directories(RayTracer PRIVATE ${RAYTRACER_INCLUDE_DIRS})

target_link_libraries(RayTracer PRIVATE PNG::PNG)

# Header files for IDEs
set(RAYTRACER_HEADERS
    src/acceleration/aabb.h
    src/acceleration/bvh.h
    src/main.h
    src/camera.h
    src/core/color.h
    src/core/interval.h
    src/core/ray.h
    src/core/vec3.h
    src/image/stb_image.h
    src/image/rtw_stb_image.h
    src/image/wpng.h
    src/materials/material.h
    src/materials/textures/perlin.h
    src/materials/textures/texture.h
    src/materials/volumes/constant_medium.h
    src/objects/hittable.h
    src/objects/hittable_list.h
    src/objects/primitives/quad.h
    src/objects/primitives/sphere.h
)

# Make headers visible in IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src FILES ${RAYTRACER_HEADERS})
target_sources(RayTracer PRIVATE ${RAYTRACER_HEADERS})