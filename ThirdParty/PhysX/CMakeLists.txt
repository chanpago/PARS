cmake_minimum_required(VERSION 3.25)
project(PARSPhysX LANGUAGES C CXX)

if(NOT MSVC OR NOT CMAKE_GENERATOR_PLATFORM STREQUAL "x64")
    message(FATAL_ERROR "PARS currently integrates PhysX for Visual Studio / Windows x64 only.")
endif()

set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/physx" CACHE PATH "PhysX source root" FORCE)
set(TARGET_BUILD_PLATFORM windows CACHE STRING "PhysX platform" FORCE)
set(PX_OUTPUT_ARCH x86 CACHE STRING "PhysX architecture family" FORCE)
set(PX_OUTPUT_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Library output root" FORCE)
set(PX_OUTPUT_BIN_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Binary output root" FORCE)
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Static SDK" FORCE)
set(NV_USE_STATIC_WINCRT OFF CACHE BOOL "Match MiniEngine's DLL CRT" FORCE)
set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "Use debug CRT in Debug" FORCE)
set(PX_BUILDSNIPPETS OFF CACHE BOOL "No samples" FORCE)
set(PX_BUILDPVDRUNTIME OFF CACHE BOOL "No OmniPVD runtime" FORCE)
set(PX_GENERATE_GPU_PROJECTS OFF CACHE BOOL "CPU physics" FORCE)
set(PX_GENERATE_GPU_PROJECTS_ONLY OFF CACHE BOOL "CPU physics" FORCE)
set(PX_COPY_EXTERNAL_DLL OFF CACHE BOOL "No external runtime DLLs" FORCE)
set(PX_USE_NVTX OFF CACHE BOOL "No NVTX dependency" FORCE)

add_subdirectory(physx/compiler/public sdk)

set(PARS_PHYSX_LIBRARIES
    PhysX PhysXCommon PhysXFoundation PhysXCooking
    PhysXExtensions PhysXCharacterKinematic PhysXPvdSDK)
foreach(sdk_target IN LISTS PARS_PHYSX_LIBRARIES)
    # Stable paths shared with PhysX.props, independent of upstream output naming.
    foreach(config DEBUG CHECKED PROFILE RELEASE)
        string(TOLOWER "${config}" config_dir)
        set_target_properties(${sdk_target} PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY_${config} "${CMAKE_CURRENT_BINARY_DIR}/lib/${config_dir}"
            COMPILE_PDB_OUTPUT_DIRECTORY_${config} "${CMAKE_CURRENT_BINARY_DIR}/lib/${config_dir}")
    endforeach()
endforeach()
add_custom_target(PARSPhysXSDK DEPENDS ${PARS_PHYSX_LIBRARIES})

add_library(PARSPhysX INTERFACE)
target_include_directories(PARSPhysX INTERFACE "${PHYSX_ROOT_DIR}/include")
target_compile_definitions(PARSPhysX INTERFACE PX_PHYSX_STATIC_LIB)
target_link_libraries(PARSPhysX INTERFACE ${PARS_PHYSX_LIBRARIES})

# The smoke test uses the same CRT and SDK libraries as MiniEngine.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:debug>:Debug>DLL")
set(CMAKE_CXX_FLAGS_PROFILE "/O2 /DNDEBUG /Zi")
set(CMAKE_CXX_FLAGS_CHECKED "/O2 /DNDEBUG /Zi")
set(CMAKE_EXE_LINKER_FLAGS_PROFILE "/DEBUG /INCREMENTAL:NO")
set(CMAKE_EXE_LINKER_FLAGS_CHECKED "/DEBUG /INCREMENTAL:NO")
add_executable(PARSPhysXSmoke EXCLUDE_FROM_ALL tests/SmokeTest.cpp)
target_link_libraries(PARSPhysXSmoke PRIVATE PARSPhysX)
set_target_properties(PARSPhysXSmoke PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/$<CONFIG>")
enable_testing()
add_test(NAME PhysXSmoke COMMAND PARSPhysXSmoke)
