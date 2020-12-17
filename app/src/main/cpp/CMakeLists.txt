
cmake_minimum_required(VERSION 3.10.2)


project("demo")

add_library(
        native-lib
        SHARED
        native-lib.cpp)
# ffmpeg 静态库存放目录
set(ffmpeg_static_path ${CMAKE_SOURCE_DIR}/ffmpeg/${CMAKE_ANDROID_ARCH_ABI})
# faac 静态库的存放目录
set(faac_static_path ${CMAKE_SOURCE_DIR}/faac/${CMAKE_ANDROID_ARCH_ABI})
# x264.a静态库的存放目录
set(x264_static_path ${CMAKE_SOURCE_DIR}/x264/${CMAKE_ANDROID_ARCH_ABI})
# 设置头文件目录，设置后系统会默认在投文件中查找相关库
include_directories(include)

# 添加编译命令
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L${ffmpeg_static_path} -L${faac_static_path} -L${x264_static_path} ")

find_library(
        log-lib
        log)


target_link_libraries(
        native-lib
        # 链接ffmpeg静态库
        avformat avcodec avfilter avutil swresample swscale avdevice
        # 链接faac静态库
        faac
        # 链接x264静态库
        x264
        ${log-lib})