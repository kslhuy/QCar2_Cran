cmake_minimum_required(VERSION 3.18)
project(cran_electronics_core LANGUAGES CXX)

option(CRAN_BUILD_SHARED "Build the desktop SIL shared library" ON)
option(CRAN_BUILD_STATIC "Build a portable MCU/SoC static library" ON)

set(
    CRAN_CORE_SOURCES
    cran_core.cpp
    cran_hil_protocol.cpp
    cran_target_adapter.cpp
    trust_observer_core.cpp
)

function(cran_configure_target target_name)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    target_include_directories(${target_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

if(CRAN_BUILD_SHARED)
    add_library(cran_electronics_core SHARED ${CRAN_CORE_SOURCES})
    target_compile_definitions(cran_electronics_core PRIVATE CRAN_CORE_BUILD=1)
    set_target_properties(cran_electronics_core PROPERTIES PREFIX "")
    cran_configure_target(cran_electronics_core)

    if(MINGW)
        # Produce a self-contained DLL that ctypes can load without modifying PATH.
        target_link_options(cran_electronics_core PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
endif()

if(CRAN_BUILD_STATIC)
    add_library(cran_electronics_core_static STATIC ${CRAN_CORE_SOURCES})
    target_compile_definitions(cran_electronics_core_static PUBLIC CRAN_CORE_STATIC=1)
    cran_configure_target(cran_electronics_core_static)
endif()
