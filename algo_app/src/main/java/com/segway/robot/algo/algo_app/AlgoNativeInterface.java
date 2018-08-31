package com.segway.robot.algo.algo_app;

import android.graphics.Bitmap;
import android.graphics.Rect;

/**
 * Created by ark338 on 2016/11/23.
 */

public class AlgoNativeInterface {

    static {
        System.loadLibrary("AlgoBase");
        System.loadLibrary("Algo");
    }

    public static native String native_readSysfs(String path);

    public static native void init(Algo algo);

    public static native void testAlgoStart(boolean isSim);

    public static native void testAlgoExit();

    public static native void recordAlgoStart(boolean isTmp);

    public static native void recordAlgoStop();

    public static native boolean nativeShow(Bitmap bitmap);

    public static native boolean isOpenglView();

    public static native String getDebugInfo();

    public static native void newRect(Rect rect);

    public static native void funcA();

    public static native void funcB();

    public static native void funcC();

    public static native void remoteControl(float v, float w);

    // Main render loop.
    public static native void render();

    // Allocate OpenGL resources for rendering.
    public static native void onGlSurfaceCreated();

    public static native void nativeOnTouchEvent(int touchCount, int event0,
                                                 float x0, float y0, float x1, float y1);

    // Setup the view port width and height.
    public static native void setupGraphic(int width, int height);

    public static native int CameraConfigureType();

    public static native void nativeHeadTouchEvent(int event);

    public static native void nativeIsEmergencyStop(boolean event);

    public static native void nativeOnWheelSliding(int event);

}
