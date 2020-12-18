package com.ffmpeg.demo;

import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import androidx.annotation.NonNull;

public class VPlayer implements SurfaceHolder.Callback {
    static {
        System.loadLibrary("native-lib");
    }

    private native void native_prepare(String dataSource);


    private native void native_start();

    private native void native_set_surface(Surface surface);

    private SurfaceHolder surfaceHolder;

    public void setSurfaceHolder(SurfaceView surfaceView) {
        if(surfaceHolder != null){
            this.surfaceHolder.removeCallback(this);
        }
        this.surfaceHolder = surfaceView.getHolder();
        this.surfaceHolder.addCallback(this);
    }

    @Override
    public void surfaceCreated(@NonNull SurfaceHolder holder) {
        // surface创建成功后

    }

    @Override
    public void surfaceChanged(@NonNull SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(@NonNull SurfaceHolder holder) {

    }
}
