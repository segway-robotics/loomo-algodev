package com.segway.robot.algo.algo_app;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Rect;
import android.os.Build;
import android.provider.Settings;
import android.util.Log;

import com.alibaba.fastjson.JSONArray;
import com.alibaba.fastjson.JSONObject;
import com.segway.robot.algo.Utils;
import com.segway.robot.algo.algo_app.wifiap.WifiApManager;
import com.segway.robot.algo.module.MusicPlayer;
import com.segway.robot.algo.module.RobotBaseImpl;
import com.segway.robot.algo.module.locomotion.LocomotionProxy;
import com.segway.sdk.nft.NFTAssistant;
import com.segway.sdk.nft.download.DownLoadObserver;
import com.segway.sdk.nft.download.DownloadInfo;
import com.segway.sdk.nft.other.Parameter;
import com.segway.sdk.nft.upload.SubmitResult;
import com.segway.sdk.nft.upload.UploadProgressListener;
import com.segway.sdk.nft.upload.UploadResult;
import com.segway.sdk.nft.util.FileUtil;

import java.io.IOException;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by ark338 on 2016/11/23.
 */
public class Algo {
    private static final String TAG = "AlgoJava";

    private static Algo ourInstance = new Algo();
    private Context mContext;
    private MusicPlayer mMusicPlayer;

    public static Algo getInstance() {
        return ourInstance;
    }
    private Algo() {
        AlgoNativeInterface.init(this);
    }

    public void init(Context context){
        mContext = context.getApplicationContext();
    }

    public void start(boolean isSim) {
        mMusicPlayer = MusicPlayer.getInstance();
        mMusicPlayer.initialize(mContext);
        AlgoNativeInterface.testAlgoStart(isSim);
    }

    public void stop() {
        AlgoNativeInterface.testAlgoExit();
        mMusicPlayer.release();
    }

    public void startRecording(boolean isTmp) {
        AlgoNativeInterface.recordAlgoStart(isTmp);
    }

    public void stopRecording() {
        AlgoNativeInterface.recordAlgoStop();
    }

    public void newRect(Rect rect) {
        AlgoNativeInterface.newRect(rect);
    }

    public void funcA() {
        AlgoNativeInterface.funcA();
    }

    public void funcB() {
        AlgoNativeInterface.funcB();
    }

    public void funcC() {
        AlgoNativeInterface.funcC();
    }

    public void remoteControl(float v, float w) {
        AlgoNativeInterface.remoteControl(v,w);
    }

    public String getDebugInfo() {
        return AlgoNativeInterface.getDebugInfo();
    }

    // What you see below is some ugly legacy api.
    // These api should be put in different function module.

    public String getSysFsFromNative(String path) {
        return AlgoNativeInterface.native_readSysfs(path);
    }

    public void render() {
        AlgoNativeInterface.render();
    }

    public boolean renderBitmapByNative(Bitmap bitmap) {
        return AlgoNativeInterface.nativeShow(bitmap);
    }

    public boolean isOpenGLView() {
        return AlgoNativeInterface.isOpenglView();
    }

    public void onOpenGLSurfaceCreated() {
        AlgoNativeInterface.onGlSurfaceCreated();
    }

    public void onTouchEventToNative(int touchCount, int event0,
                                   float x0, float y0, float x1, float y1) {
        AlgoNativeInterface.nativeOnTouchEvent(touchCount, event0, x0, y0, x1, y1);
    }

    public void setupGraphic(int width, int height) {
        AlgoNativeInterface.setupGraphic(width, height);
    }

    public int CameraConfigureType() {
        return AlgoNativeInterface.CameraConfigureType();
    }

    private static final String SOUND_WARN1 = "glass";
    private static final String SOUND_WARN2 = "iot";

    private long lastPlayWavFinishTime = 0;
    private int lastPlayWavLength = 0;

    public void playSoundWarning(int id, int soundLengthMillis) {
        if(System.currentTimeMillis()-lastPlayWavFinishTime<lastPlayWavLength)
            return;
        else{
            lastPlayWavFinishTime = System.currentTimeMillis();
            lastPlayWavLength = soundLengthMillis;
        }

        if (id == 0)
            mMusicPlayer.playMusic(SOUND_WARN1);
        else if (id == 1)
            mMusicPlayer.playMusic(SOUND_WARN2);
    }

    public void headTouchEvent(int event){
        AlgoNativeInterface.nativeHeadTouchEvent(event);
    }

    public boolean isPowerON(String device_id){
        String result = CommandExecutor.exeCmd("adb -s "+ device_id + " shell dumpsys power | grep -s Display | grep -s Power | grep -s state");
        if(result.length()>0) {
            String[] key_values = result.split("=");
            if (key_values.length == 2) {
                String value = key_values[1];
                if (value.equals("ON"))
                    return true;
                else
                    return false;
            }
        }

        return false;
    }

    public boolean connectTango() {
        CommandExecutor.exeCmd("adb kill-server");
        String [] cmd = {"/system/bin/sh" , "-c", "adb devices| grep -v emulator | grep -v attach | grep -v daemon"};
        String result = CommandExecutor.exeCmd(cmd);
        String deviceid = null;
        String[] deviceids = null;
        if(result.length()>0){
            deviceids = result.split("\t");
            if(deviceids.length==2) {
                deviceid = deviceids[0];
                if(!isPowerON(deviceid)){
                    CommandExecutor.exeCmd("adb -s "+ deviceid + " shell input keyevent 26");
                    CommandExecutor.exeCmd("adb -s "+ deviceid + " shell input touchscreen swipe 930 880 930 380");
                }
                CommandExecutor.exeCmd("adb -s "+ deviceid + " shell am start -n com.projecttango.examples.cpp.pointcloud/.StartActivity");
                CommandExecutor.exeCmd("adb -s "+ deviceid + " forward tcp:8086 tcp:8086");
                CommandExecutor.exeCmd("adb -s "+ deviceid + " shell am broadcast -a startTango --es action loadAreaMapping");
                return true;
            }
        }

        //CommandExecutor.exeCmd("cat /data/.android/adbkey");
        return false;

    }

    public boolean disconnectTango() {
        CommandExecutor.exeCmd("adb kill-server");
        String [] cmd = {"/system/bin/sh" , "-c", "adb devices| grep -v emulator | grep -v attach | grep -v daemon"};
        String result = CommandExecutor.exeCmd(cmd);
        String deviceid = null;
        String[] deviceids = null;
        if(result.length()>0){
            deviceids = result.split("\t");
            if(deviceids.length==2) {
                deviceid = deviceids[0];
                CommandExecutor.exeCmd("adb -s "+ deviceid + " shell am force-stop com.projecttango.examples.cpp.pointcloud");
                if(isPowerON(deviceid)){
                    CommandExecutor.exeCmd("adb -s "+ deviceid + " shell input keyevent 26");
                }

                return true;
            }
        }

        return false;
    }

    public void isEmergencyStop(boolean is_stop) {
        if(Utils.getRobotModel() == 2){
            is_stop = !is_stop;
            if(!is_stop) {
                RobotBaseImpl mRobotBaseImpl = (RobotBaseImpl) LocomotionProxy.getInstance().getBaseImpl();
                mRobotBaseImpl.setBaseCloseLoop(true);
            }
        }
        AlgoNativeInterface.nativeIsEmergencyStop(is_stop);
        return;
    }

    public String getDeviceId(){
        return Build.SERIAL;
    }

    public boolean uploadRawMap(String rawmap_path){
        final String f_rawmap_path = rawmap_path;
        final NFTAssistant assistant = new NFTAssistant();
        assistant.setParams(Parameter.ACCESS_KEY_ID, "5401760877819")
                .setParams(Parameter.SECRET_ACCESS_KEY_ID, "yCoIiMaxJl6nPvG6kmV9Tw==")
                .setParams(Parameter.BUCKET_NAME, "-apr-v2-raw-map")
                .init();
        final AtomicBoolean uploadRet = new AtomicBoolean(false);// false: not finish, true: complete and successful
        new Thread(new Runnable() {
            @Override
            public void run() {
                // TODO filename is uuid
                UploadResult uploadResult = assistant.uploadDirectory(f_rawmap_path, new UploadProgressListener() {
                    @Override
                    public void onProgress(String fileName, int done, int sum) {
                        Log.d(TAG, "onProgress() called with: fileName = [" + fileName + "], done = [" + done + "], sum = [" + sum + "]");
                    }
                });
                if (uploadResult != null) {
                    Log.i(TAG, "uploadRawMap: url = " + uploadResult.url + ", md5 = " + uploadResult.md5);
                }
                JSONArray rawMapList = new JSONArray();
                JSONObject rawMap = new JSONObject();
                rawMap.put("mapUrl", uploadResult.url);
                rawMap.put("md5", uploadResult.md5);
                rawMap.put("mapName", "test_map");
                rawMap.put("mapType", "raw");
                JSONObject gpsinfo = new JSONObject();
                gpsinfo.put("alt", 0.0);
                gpsinfo.put("lat", 0.0);
                gpsinfo.put("lng", 0.0);
                rawMap.put("gps", gpsinfo);
                rawMapList.add(rawMap);
                JSONObject build_rawmap_result = new JSONObject();
                build_rawmap_result.put("robotId", Build.SERIAL);
                build_rawmap_result.put("rawMapList", rawMapList);

                boolean isSuccess = false;
                try {
                    SubmitResult submitResult = assistant.submit("http://nav-v3.loomo.com/navigation/v2/map/build", build_rawmap_result.toString());
                    isSuccess = submitResult.isSuccessful();
                    Log.d(TAG,"submitResult =  " + submitResult);
                } catch (IOException e) {
                    e.printStackTrace();
                    isSuccess = false;
                }
                Log.d(TAG,"uploadRawMap url " + uploadResult.url);

                synchronized (uploadRet) {
                    uploadRet.set(isSuccess);
                    uploadRet.notify();
                }
            }
        }).start();

        synchronized (uploadRet) {
            if (!uploadRet.get()) {
                try {
                    //1 hour timeout
                    Log.i(TAG, "uploadRawMap wait");
                    uploadRet.wait(60 * 60 * 1000);
                    Log.i(TAG, "uploadRawMap exit");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        return uploadRet.get();
    }

    public boolean uploadMapFile(String map_file){
        final String f_map_file = map_file;
        final NFTAssistant assistant = new NFTAssistant();
        assistant.setParams(Parameter.ACCESS_KEY_ID, "5401760877819")
                .setParams(Parameter.SECRET_ACCESS_KEY_ID, "yCoIiMaxJl6nPvG6kmV9Tw==")
                .setParams(Parameter.BUCKET_NAME, "-apr-v2-raw-map")
                .init();
        final AtomicBoolean uploadRet = new AtomicBoolean(false);// false: not finish, true: complete and successful
        new Thread(new Runnable() {
            @Override
            public void run() {
                UploadResult uploadResult = assistant.uploadFile(f_map_file, new UploadProgressListener() {
                    @Override
                    public void onProgress(String fileName, int done, int sum) {
                        Log.d(TAG, "onProgress() called with: fileName = [" + fileName + "], done = [" + done + "], sum = [" + sum + "]");
                    }
                });

                boolean isSuccess = false;
                if (uploadResult != null) {
                    Log.i(TAG, "uploadMapFile: url = " + uploadResult.url + ", md5 = " + uploadResult.md5);
                    isSuccess = true;
                }

                synchronized (uploadRet) {
                    uploadRet.set(isSuccess);
                    uploadRet.notify();
                }
            }
        }).start();

        synchronized (uploadRet) {
            if (!uploadRet.get()) {
                try {
                    //1 hour timeout
                    Log.i(TAG, "uploadMapFile wait");
                    uploadRet.wait(60 * 60 * 1000);
                    Log.i(TAG, "uploadMapFile exit");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        return uploadRet.get();
    }


    public boolean downloadOptimizedMap(String optimizedmap_url, String md5_sum,
                                        String save_path, String filename) {
        Log.d(TAG,"downloadOptimizedMap" + optimizedmap_url + md5_sum + save_path + filename);
        final String f_optimizedmap_url = optimizedmap_url;
        final String f_md5_sum = md5_sum;
        final String f_save_path = save_path;
        final String f_filename = filename;
        final NFTAssistant assistant = new NFTAssistant();
        assistant.setParams(Parameter.ACCESS_KEY_ID, "5401760877819")
                .setParams(Parameter.SECRET_ACCESS_KEY_ID, "yCoIiMaxJl6nPvG6kmV9Tw==")
                .setParams(Parameter.BUCKET_NAME, "-apr-v2-processed-map")
                .init();
        final AtomicBoolean downRet = new AtomicBoolean(false);// false: not finish, true: complete and successful
        new Thread(new Runnable() {
            @Override
            public void run() {
                DownLoadObserver downLoadObserver = new DownLoadObserver() {
                    @Override
                    public void onNext(DownloadInfo downloadInfo) {
                        super.onNext(downloadInfo);
                        long progress = downloadInfo.getProgress();
                        long total = downloadInfo.getTotal();
                        Log.i(TAG, "onNext progress: " + progress + ", total: " + total);
                    }

                    @Override
                    public void onComplete() {
                        Log.i(TAG, "onComplete: ");
                        synchronized (downRet) {
                            downRet.set(true);
                            downRet.notify();
                        }
                    }

                    @Override
                    public void onError(Throwable e) {
                        Log.d(TAG, "onError() called with: e = [" + e + "]");
                        synchronized (downRet) {
                            downRet.set(false);
                            downRet.notify();
                        }
                    }
                };
                assistant.downloadFile(f_optimizedmap_url, f_save_path + "/" + f_filename, f_md5_sum, downLoadObserver);
            }
        }).start();
        synchronized (downRet) {
            if (!downRet.get()) {
                try {
                    //1 hour timeout
                    Log.i(TAG, "downloadOptimizedMap wait");
                    downRet.wait(60 * 60 * 1000);
                    Log.i(TAG, "downloadOptimizedMap exit");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        Log.d(TAG,"downloadOptimizedMap");
        return downRet.get();
    }

    public void enableWifiAp(){
        new Thread() {
            @Override
            public void run() {
                super.run();
                WifiApManager.getInstance().initialize(mContext, null);
            }
        }.start();
    }
	public void onWheelSliding(int slidingEvent){
	AlgoNativeInterface.nativeOnWheelSliding(slidingEvent);
	}
}
