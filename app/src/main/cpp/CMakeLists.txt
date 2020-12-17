
cmake_minimum_required(VERSION 3.10.2)


project("demo")

add_library(
        native-lib
        SHARED
        native-lib.cpp)

set(ffmpeg_static_path ${CMAKE_SOURCE_DIR}/ffmpeg/${CMAKE_ANDROID_ARCH_ABI})
set(faac_static_path ${CMAKE_SOURCE_DIR}/faac/${CMAKE_ANDROID_ARCH_ABI})
set(x264_static_path ${CMAKE_SOURCE_DIR}/x264/${CMAKE_ANDROID_ARCH_ABI})
include_directories(include)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L${ffmpeg_static_path} -L${faac_static_path} -L${x264_static_path} ")

find_library(
        log-lib
        log)


target_link_libraries(
        native-lib
        avformat avcodec avfilter avutil swresample swscale
        faac
        x264
        ${log-lib})