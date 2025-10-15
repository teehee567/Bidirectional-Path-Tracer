cmake_minimum_required(VERSION 3.10)
project(RayTracer)
set(CMAKE_CXX_STANDARD 17)

find_package(PNG REQUIRED)

add_executable(RayTracer
    src/main.cpp
    src/aabb.h
    src/bvh.h
    src/camera.h
    src/color.h
    src/constant_medium.h
    src/hittable.h
    src/hittable_list.h
    src/interval.h
    src/main.h
    src/material.h
    src/perlin.h
    src/quad.h
    src/ray.h
    src/rtw_stb_image.h
    src/sphere.h
    src/texture.h
    src/vec3.h
    src/wpng.h
)

target_link_libraries(RayTracer PRIVATE PNG::PNG)