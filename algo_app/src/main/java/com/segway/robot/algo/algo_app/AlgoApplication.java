package com.segway.robot.algo.algo_app;

import android.app.ActivityManager;
import android.app.Application;
import android.content.Context;
import android.util.Log;

import com.segway.robot.sdk.base.classloader.NativeLibraryLoader;

public class AlgoApplication extends Application{
    private static final String TAG = "AlgoApplication";
    private static final String PROCESS_NAME_NEED_LOAD_LIB = "com.segway.robot.algo.algo_app";

    private static Context mContext;

    public static Context getContext() {
        return mContext;
    }

    boolean checkMainProcessName(String process_name){
        int pid = android.os.Process.myPid();
        String curProcessName = "";
        ActivityManager manager = (ActivityManager) getApplicationContext().getSystemService(Context.ACTIVITY_SERVICE);
        for (ActivityManager.RunningAppProcessInfo process: manager.getRunningAppProcesses()) {
            if(process.pid == pid){
                curProcessName = process.processName;
            }
        }
        Log.d(TAG, "checkMainProcessName: " + curProcessName);
        if(process_name.equals(curProcessName))
            return true;
        else
            return false;
    }

    @Override
    public void onCreate() {
        super.onCreate();

        if(checkMainProcessName(PROCESS_NAME_NEED_LOAD_LIB))
            NativeLibraryLoader.load(this);
        mContext = getApplicationContext();

        Log.d(TAG, "onCreate() called");
        new Thread() {
            @Override
            public void run() {
                super.run();
                Log.d(TAG, "run() called");
            }
        }.start();
    }
}
