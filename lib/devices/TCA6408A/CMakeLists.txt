cmake_minimum_required(VERSION 3.16)

# Project name
project(tca6408a)

# Define the library
add_library(tca6408a STATIC
    tca6408a.c
)

# Conditionally include platform-specific source files
if(ESP_PLATFORM)
    target_sources(tca6408a PRIVATE platform/tca6408a_espidf.c)
    target_compile_definitions(tca6408a PRIVATE ESP_PLATFORM)
endif()

# Include directories
target_include_directories(tca6408a PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tca6408a PRIVATE platform)
