#include <jni.h>
#include <string>


extern "C"{
#include <libavcodec/avcodec.h>
#include <libfaac/faaccfg.h>
#include <libx264/x264.h>
#include <libavcodec/avcodec.h>
}

//extern "C" JNIEXPORT jstring JNICALL
//Java_com_ffmpeg_demo_MainActivity_stringFromJNI(
//        JNIEnv* env,
//        jobject /* this */) {
//    std::string hello = "Hello from C++";
//    faacEncConfiguration faacEncConfiguration;
//
//    return env->NewStringUTF(av_version_info());
//}

//ANativeWindow * window;

extern "C"
JNIEXPORT void JNICALL
Java_com_ffmpeg_demo_VPlayer_native_1prepare(JNIEnv *env, jobject thiz, jstring data_source) {
    // TODO: implement native_prepare()
}


extern "C"
JNIEXPORT void JNICALL
Java_com_ffmpeg_demo_VPlayer_native_1start(JNIEnv *env, jobject thiz) {
    // TODO: implement native_start()
}


extern "C"
JNIEXPORT void JNICALL
Java_com_ffmpeg_demo_VPlayer_native_1set_1surface(JNIEnv *env, jobject thiz, jobject surface) {
    // TODO: implement native_set_surface()
}