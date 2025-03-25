cmake_minimum_required(VERSION 3.5)
project(camcontrol)

set(CMAKE_C_FLAGS "-Wall -Wextra -O3")

find_package(PNG REQUIRED)
find_package( Threads REQUIRED )
find_library(GXIAPI_LIBRARY names gxiapi PATHS ENV GALAXY_SDK_PATH REQUIRED)
find_path(GXIAPI_INCLUDE_PATH "GxIAPI.h" PATHS ENV GALAXY_SDK_PATH PATH_SUFFIXES "inc" REQUIRED)

find_package(PkgConfig REQUIRED) 
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
add_definitions(${GTK3_CFLAGS_OTHER})

set(SOURCE_FILES 
    "main.c"
    "pixel.c"
    "utils.c")

include_directories(${PROJECT_NAME} ${GXIAPI_INCLUDE_PATH} ${PNG_INCLUDE_DIR} ${GTK3_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Threads::Threads ${GXIAPI_LIBRARY} ${PNG_LIBRARY} ${GTK3_LIBRARIES})