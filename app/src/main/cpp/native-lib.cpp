#include <jni.h>
#include <string>


extern "C"{
#include <libavcodec/avcodec.h>
#include <libfaac/faaccfg.h>
#include <libx264/x264.h>
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_ffmpeg_demo_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    faacEncConfiguration faacEncConfiguration;

    return env->NewStringUTF(av_version_info());
}