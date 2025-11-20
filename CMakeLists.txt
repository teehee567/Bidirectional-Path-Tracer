cmake_minimum_required(VERSION 3.10)
project(RayTracer)
set(CMAKE_CXX_STANDARD 17)

find_package(PNG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects/primatives
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects
    ${CMAKE_CURRENT_SOURCE_DIR}/src/acceleration
    ${CMAKE_CURRENT_SOURCE_DIR}/src/scene
    ${CMAKE_CURRENT_SOURCE_DIR}/src/renderers
)

target_link_libraries(RayTracer PRIVATE PNG::PNG glm::glm yaml-cpp::yaml-cpp)

# Aggressive optimization for Release mode
if(MSVC)
    # Maximum optimization flags for Release builds
    target_compile_options(RayTracer PRIVATE
        $<$<CONFIG:Release>:
            /O2              # Maximum optimization (favor speed)
            /Ob2             # Aggressive inlining
            /Oi              # Generate intrinsic functions
            /Ot              # Favor fast code
            /GL              # Whole program optimization
            /fp:fast         # Fast floating point model
            /arch:AVX2       # Use AVX2 instructions (if CPU supports it)
            /Qpar            # Auto-parallelization
            /favor:INTEL64   # Optimize for Intel 64 architecture
        >
    )
    
    # Link-time code generation for Release builds
    set_target_properties(RayTracer PROPERTIES
        $<$<CONFIG:Release>:INTERPROCEDURAL_OPTIMIZATION> TRUE
    )
    
    # Additional linker optimizations
    target_link_options(RayTracer PRIVATE
        $<$<CONFIG:Release>:
            /LTCG            # Link-time code generation
            /OPT:REF         # Remove unreferenced functions and data
            /OPT:ICF         # Identical COMDAT folding
            /INCREMENTAL:NO  # Disable incremental linking
        >
    )
endif()