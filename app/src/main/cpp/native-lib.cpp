#include <jni.h>
#include <string>
#include <libfaac/faaccfg.h>

extern "C"{
#include <stdio.h>
#include <libavcodec/avcodec.h>
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_ffmpeg_demo_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    faacEncConfiguration faacEncConfiguration;
    return env->NewStringUTF(av_version_info());
}