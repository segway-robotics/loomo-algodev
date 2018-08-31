package com.segway.robot.algo.algo_app;


import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Pair;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;

import android.widget.Button;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.segway.robot.algo.Utils;
import com.segway.robot.algo.algo_app.wifiap.WifiApManager;
import com.segway.robot.algo.module.RobotBaseImpl;
import com.segway.robot.algo.module.RobotHeadImpl;
import com.segway.robot.algo.module.RobotSensorImpl;
import com.segway.robot.algo.module.RobotExternalImpl;
import com.segway.robot.algo.module.VisionImpl;
import com.segway.robot.algo.module.locomotion.LocomotionProxy;
import com.segway.robot.algo.module.vision.VisionProxy;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.baseconnectivity.Message;
import com.segway.robot.sdk.baseconnectivity.MessageConnection;
import com.segway.robot.sdk.baseconnectivity.MessageRouter;
import com.segway.robot.sdk.connectivity.BufferMessage;
import com.segway.robot.sdk.connectivity.RobotException;
import com.segway.robot.sdk.connectivity.RobotMessageRouter;
import com.segway.robot.sdk.connectivity.StringMessage;
import com.segway.robot.sdk.locomotion.head.Head;
//import com.segway.robot.sdk.locomotion.sbv.RobotWheelSlidingListener;
import com.segway.robot.sdk.vision.DTS;
import com.segway.robot.sdk.vision.frame.Frame;
import com.segway.robot.sdk.vision.stream.FrameRate;
import com.segway.robot.sdk.vision.stream.PixelFormat;
import com.segway.robot.sdk.vision.stream.Resolution;
import com.segway.robot.sdk.vision.stream.StreamInfo;
import com.segway.robot.sdk.vision.stream.StreamType;
import com.segway.robot.algo.tf.AlgoTfData;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;

import butterknife.Bind;
import butterknife.ButterKnife;
import butterknife.OnCheckedChanged;
import butterknife.OnClick;
import timber.log.Timber;


public class MainActivity extends Activity {
    private static final String thermalpath = "sys/devices/platform/coretemp.0/temp2_input";

    // GLSurfaceView and renderer, all of the graphic content is rendered
    // through OpenGL ES 2.0 in native code.
    private Renderer mRenderer;
    // Screen size for normalizing the touch input for orbiting the render camera.
    private Point mScreenSize = new Point();

    private boolean mUseOpenglView = false;

    @Bind(R.id.bind)
    Switch mBindSwitch;

    @Bind(R.id.button4)
    Button mAlgoTestButton;

    @Bind(R.id.button5)
    Button mAlgoRecordButton;

    @Bind(R.id.button6)
    Button mRecordNewLoc;

    @Bind(R.id.button7)
    Button mLiveSimSwitch;

    @Bind(R.id.image3)
    ImageView mColorImage;

    @Bind(R.id.image1)
    ImageView mDepthImage;

    @Bind(R.id.image2)
    ImageView mFishEyeImage;

    @Bind(R.id.image4)
    ImageView mPlatformImage;

    private boolean mBind = false;
    private boolean mAlgoStarted = false;
    private boolean mRecordStarted = false;
    private boolean mRecordTmp = true;
    private boolean mIsSim = false;
    private Bitmap AlgoBitmap;
    private Rect mDrawingRect;

    private GLSurfaceView mGlSurfaceView;
    private RelativeLayout mRelativeLayout;

    private int mCameraConfig = 0;

    private VisionImpl mVisionImpl = new VisionImpl();
    private RobotHeadImpl mRobotHeadImpl = new RobotHeadImpl();
    private RobotBaseImpl mRobotBaseImpl = new RobotBaseImpl();
    private RobotSensorImpl mRobotSensorImpl = new RobotSensorImpl();
    private RobotExternalImpl mRobotExternalImpl = new RobotExternalImpl();
    private RobotMessageRouter mRobotMessageRouter = RobotMessageRouter.getInstance();
    private boolean mBindExternalSuccess = false;
    private EventReceiver mEventReceiver;
    private MessageConnection mMessageConnection = null;
    private HandlerThread mControlThread;
    private Handler mControlHandler = null;
    private static final int CONTROL_MOVE = 110;
    private static final int SPEED_MSG_LENGTH = 20;
    private static final int CONTROL_MSG_LENGTH = 1;
    private static final byte RESET_CONTROL_MSG = 1;
    private boolean isAdjusting = false;
    private float angle;

    int PLATFORM_WIDTH = 640;
    int PLATFORM_HEIGHT = 480;

    WifiApManager wifiApManager = null;

    private ServiceBinder.BindStateListener mVisionBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "Vision onBind");
            mBind = true;
            mBindSwitch.setClickable(true);
            doStartStop(true);
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "Vision onUnbind:" + reason);
            mBind = false;
            doStartStop(false);
        }
    };

    private ServiceBinder.BindStateListener mHeadBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "Head onBind");
            LocomotionProxy.getInstance().setMode(Head.MODE_SMOOTH_TACKING);
            LocomotionProxy.getInstance().resetOrientation();
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "Head onUnbind:" + reason);
        }
    };

//    private RobotWheelSlidingListener mRobotWheelSlidingListener = new RobotWheelSlidingListener() {
//        @Override
//        public void onWheelSliding(int slidingEvent) {
//            Log.d(TAG, "onWheelSliding");
//            Algo.getInstance().onWheelSliding(slidingEvent);
//        }
//    };

    private ServiceBinder.BindStateListener mBaseBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            // wheel slide
//            mRobotBaseImpl.registerWheelSlidingListener(mRobotWheelSlidingListener);
            // emergency stop on GX
            if (Utils.getRobotModel() == 2) {
                Class clazz = Algo.getInstance().getClass();
                try {
                    Method mEmergencyStop = clazz.getDeclaredMethod("isEmergencyStop", boolean.class);
                    mRobotBaseImpl.registerEmergencyStop(Algo.getInstance(), mEmergencyStop);
                    Algo.getInstance().isEmergencyStop(mRobotBaseImpl.isEmergencyStop());
                } catch (NoSuchMethodException e) {
                    e.printStackTrace();
                }
            }
            Log.d(TAG, "Base onBind");
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "Base onUnbind:" + reason);
        }
    };

    private ServiceBinder.BindStateListener mSensorBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "Sensor onBind");
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "Sensor onUnbind:" + reason);
        }
    };

    private void startBackgroundThread() {
        mControlThread = new HandlerThread("ControlThread");
        mControlThread.start();
        mControlHandler = new Handler(mControlThread.getLooper()) {
            @Override
            public void handleMessage(android.os.Message msg) {
                super.handleMessage(msg);
                Message message = (Message) msg.obj;
                byte[] bytes = (byte[]) message.getContent();
                ByteBuffer buffer = ByteBuffer.wrap(bytes);
                switch (buffer.remaining()) {
                    case SPEED_MSG_LENGTH:
                        controlStatus(buffer);
                        break;
                    case CONTROL_MSG_LENGTH:
                        if (buffer.get() == RESET_CONTROL_MSG) {
                            controlReset();
                        }
                        break;
                }
            }
        };
    }


    private void controlStatus(ByteBuffer buffer) {
        int mod = buffer.getInt();
        float[] datas = new float[4];
        for (int i = 0; i < 4; i++) {
            datas[i] = buffer.getFloat();
        }

        if (mod == Head.MODE_ORIENTATION_LOCK && mRobotHeadImpl.getMode() != Head.MODE_ORIENTATION_LOCK) {
            mRobotHeadImpl.setMode(Head.MODE_ORIENTATION_LOCK);
        }
        if (mod == Head.MODE_SMOOTH_TACKING && mRobotHeadImpl.getMode() != Head.MODE_SMOOTH_TACKING) {
            mRobotHeadImpl.setMode(Head.MODE_SMOOTH_TACKING);
        }

        mRobotBaseImpl.setLinearVelocity(datas[0]);
        mRobotBaseImpl.setAngularVelocity(-datas[1]);
        if (mRobotHeadImpl.getMode() == Head.MODE_ORIENTATION_LOCK) {
            mRobotHeadImpl.setPitchAngularVelocity(datas[2]);
            mRobotHeadImpl.setYawAngularVelocity(-datas[3]);
        } else if (mRobotHeadImpl.getMode() == Head.MODE_SMOOTH_TACKING) {
            mRobotHeadImpl.setWorldPitch(mRobotHeadImpl.getWorldPitch().getAngle() + datas[2]);

            if (datas[1] == 0 && !isAdjusting) {
                mRobotHeadImpl.setJointYaw(mRobotHeadImpl.getJointYaw().getAngle() - datas[3]);
                angle = mRobotHeadImpl.getJointYaw().getAngle();
            } else if (datas[1] != 0 && !isAdjusting) { //底盘旋转状态
                isAdjusting = true;
            } else if (datas[1] == 0 && isAdjusting) { //底盘停止旋转，头部恢复
                if (Math.abs(mRobotHeadImpl.getJointYaw().getAngle() - angle) < 0.1) {
                    isAdjusting = false;
                }
                mRobotHeadImpl.setJointYaw(angle);
            }
        }
    }

    private void controlReset() {
        mRobotHeadImpl.setMode(Head.MODE_SMOOTH_TACKING);
        mControlHandler.removeMessages(CONTROL_MOVE);
        mRobotHeadImpl.setWorldPitch(0);
        mRobotHeadImpl.setWorldYaw(0);
        mRobotBaseImpl.setAngularVelocity(0);
        mRobotBaseImpl.setLinearVelocity(0);
    }


    private void stopBackgroundThread() {
        mControlThread.quitSafely();
        try {
            mControlThread.join();
            mControlThread = null;
            mControlHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private ServiceBinder.BindStateListener mMessageRouterBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "Message router onBind");
            try {
                mRobotMessageRouter.register(mMessageConnectionListener);
            } catch (RobotException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "Message router onUnbind:" + reason);
        }
    };


    private ServiceBinder.BindStateListener mExternalBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Class clazz = Algo.getInstance().getClass();
            try {
                Method mEmergencyStop = clazz.getDeclaredMethod("isEmergencyStop", boolean.class);
                mRobotExternalImpl.registerEmergencyStop(Algo.getInstance(), mEmergencyStop);
                Algo.getInstance().isEmergencyStop(mRobotExternalImpl.isEmergencyStop());
            } catch (NoSuchMethodException e) {
                e.printStackTrace();
            }
            Log.d(TAG, "External onBind");
        }

        @Override
        public void onUnbind(String reason) {
            mRobotExternalImpl.unregisterEmergencyStop();
            Log.d(TAG, "External onUnbind:" + reason);
        }
    };

    private MessageRouter.MessageConnectionListener mMessageConnectionListener = new MessageRouter.MessageConnectionListener() {
        @Override
        public void onConnectionCreated(MessageConnection connection) {
            Log.d(TAG, "onConnectionCreated: " + connection.getName());
            mMessageConnection = connection;
            try {
                mMessageConnection.setListeners(mConnectionStateListener, mMessageListener);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    MessageConnection.ConnectionStateListener mConnectionStateListener = new MessageConnection.ConnectionStateListener() {
        @Override
        public void onOpened() {
            Log.d(TAG, "onOpened: ");
            //startBackgroundThread();
        }

        @Override
        public void onClosed(String error) {
            Log.d(TAG, "onClosed: " + error);
            //stopBackgroundThread();
        }
    };

    private MessageConnection.MessageListener mMessageListener = new MessageConnection.MessageListener() {
        @Override
        public void onMessageSentError(Message message, String error) {
            Log.d(TAG, "onMessageSentError() called with: message = [" + message + "], error = [" + error + "]");
        }

        @Override
        public void onMessageSent(Message message) {
            Log.d(TAG, "onMessageSent() called with: message = [" + message + "]");
        }

        @Override
        public void onMessageReceived(Message message) {
            receiveMessage(message);
        }
    };


    private void receiveMessage(Message message) {
        //Log.d(TAG, "receiveMessage() called with: message = [" + message + "]");
        if (message instanceof BufferMessage) {
            byte[] bytes = (byte[]) message.getContent();
            ByteBuffer buffer = ByteBuffer.wrap(bytes);
            switch (buffer.remaining()) {
                case SPEED_MSG_LENGTH:
                    int mod = buffer.getInt();
                    float v = buffer.getFloat();
                    float w = buffer.getFloat();
                    Algo.getInstance().remoteControl(v,-w);
                    //Log.d(TAG, "remote control speed = " + v + ", " + w);
                    break;
            }
            // test in activity
            if(mControlHandler!=null){
                mControlHandler.removeMessages(CONTROL_MOVE);
                android.os.Message msg = mControlHandler.obtainMessage(CONTROL_MOVE, message);
                mControlHandler.sendMessage(msg);
            }
        } else {
            Log.e(TAG, "Received StringMessage. It's not gonna happen");
        }
    }

    private EventReceiver.EventListener mEventListener = new EventReceiver.EventListener() {
        @Override
        public void onNewEvent(int event) {
            Algo.getInstance().headTouchEvent(event);
        }
    };

    @OnCheckedChanged(R.id.bind)
    void bind(boolean isChecked) {
        if (isChecked) {
            if (!mRobotHeadImpl.bindService(this.getApplicationContext(), mHeadBindStateListener)) {
                //throw new IllegalStateException("Bind robot head service failed!");
            }

            if (!mRobotBaseImpl.bindService(this.getApplicationContext(), mBaseBindStateListener)) {
                throw new IllegalStateException("Bind robot base service failed!");
            }

            if (!mRobotSensorImpl.bindService(this.getApplicationContext(), mSensorBindStateListener)) {
                throw new IllegalStateException("Bind robot sensor service failed!");
            }

            if (!mRobotMessageRouter.bindService(this.getApplicationContext(), mMessageRouterBindStateListener)) {
                throw new IllegalStateException("Bind robot message router service failed!");
            }

            if (Utils.getRobotModel() == 1) {
                Log.d(TAG, "Loomogo, start binding external sensors");
                if (!mRobotExternalImpl.bindService(this.getApplicationContext(), mExternalBindStateListener)) {
                    mBindExternalSuccess = false;
                    Log.d(TAG, "Bind robot external service failed");
                } else {
                    mBindExternalSuccess = true;
                    Log.d(TAG, "Bind robot external service success");
                }
            }

            if (!mVisionImpl.bindService(this, mVisionBindStateListener)) {
                mBindSwitch.setChecked(false);
                Toast.makeText(this, "Bind vision service failed", Toast.LENGTH_SHORT).show();
            }
            mBindSwitch.setClickable(false);
            Toast.makeText(this, "Bind vision service success", Toast.LENGTH_SHORT).show();
        } else {
            doStartStop(false);
            mVisionImpl.unbindService();
            mRobotHeadImpl.unbindService();
            mRobotBaseImpl.unbindService();
            mRobotSensorImpl.unbindService();
            if (mBindExternalSuccess)
                mRobotExternalImpl.unbindService();
            mBind = false;
            Toast.makeText(this, "Unbind vision service", Toast.LENGTH_SHORT).show();
        }
    }

    void doStartStop(boolean isChecked) {
        if (isChecked) {
            // TODO: change configuration is not recommended, disable now. If really enabled, revert configuration later (see below)
            /*mVisionImpl.cleanStream();
            if((mCameraConfig&1)!=0)
                mVisionImpl.enableStream(new StreamInfo(StreamType.DEPTH, 320, 240, FrameRate.FPS_30, PixelFormat.Z16));
            if((mCameraConfig&2)!=0)
                mVisionImpl.enableStream(new StreamInfo(StreamType.COLOR, 640, 480, FrameRate.FPS_30, PixelFormat.RGBA8888));
            if((mCameraConfig&4)!=0)
                mVisionImpl.enableStream(new StreamInfo(StreamType.FISH_EYE, 640, 480, FrameRate.FPS_30, PixelFormat.A8));
            if((mCameraConfig&2)==0 && (mCameraConfig&16)!=0)
                mVisionImpl.enableStream(new StreamInfo(StreamType.COLOR, 320, 240, FrameRate.FPS_30, PixelFormat.RGBA8888));
             if(Utils.getRobotModel() == 2) {
                mVisionImpl.enableStream(StreamInfo.create(StreamType.EXT_DEPTH_L, Resolution.Depth.R_480_270, FrameRate.FPS_15, PixelFormat.Z16));
                mVisionImpl.enableStream(StreamInfo.create(StreamType.EXT_DEPTH_R, Resolution.Depth.R_480_270, FrameRate.FPS_15, PixelFormat.Z16));
            }*/

            if ((mCameraConfig & 1) != 0) {
                Log.d(TAG, "DEPTH stream enabled");
                mVisionImpl.startImageTransferMemoryFileBuffer(StreamType.DEPTH);
            }
            if ((mCameraConfig & 2) != 0 || (mCameraConfig & 16) != 0) {
                Log.d(TAG, "COLOR stream enabled");
                mVisionImpl.startImageTransferMemoryFileBuffer(StreamType.COLOR);
            }
            if ((mCameraConfig & 4) != 0) {
                Log.d(TAG, "FISH_EYE stream enabled");
                mVisionImpl.startImageTransferMemoryFileBuffer(StreamType.FISH_EYE);
            }

            if ((mCameraConfig & 8) != 0) {
                Log.d(TAG, "PlatformCamera stream enabled");
                mVisionImpl.enablePlatformCamera(this);
            }

            if(Utils.getRobotModel() == 2) {
                mVisionImpl.startExtDepthSteam();
                mRobotBaseImpl.setBaseCloseLoop(true);
            }

            startAlgo();
//            startVisionSample();
//            startControlSample();
        } else {
            stopAlgo();
//            stopVisionSample();
//            stopControlSample();

            // API: enable stream from service to sdk buffer
            if ((mCameraConfig & 1) != 0)
                mVisionImpl.stopImageTransferMemoryFileBuffer(StreamType.DEPTH);
            if ((mCameraConfig & 2) != 0)
                mVisionImpl.stopImageTransferMemoryFileBuffer(StreamType.COLOR);
            if ((mCameraConfig & 4) != 0)
                mVisionImpl.stopImageTransferMemoryFileBuffer(StreamType.FISH_EYE);

            if ((mCameraConfig & 8) != 0)
                mVisionImpl.disablePlatformCamera();

            if(Utils.getRobotModel() == 2){
                mRobotBaseImpl.setBaseCloseLoop(false);
                mVisionImpl.stopExtDepth();
            }

            // TODO: revert configuration if needed
            /*mVisionImpl.enableStream(new StreamInfo(StreamType.DEPTH, 320, 240, FrameRate.FPS_30, PixelFormat.Z16));
            mVisionImpl.enableStream(new StreamInfo(StreamType.COLOR, 640, 480, FrameRate.FPS_30, PixelFormat.RGBA8888));
            mVisionImpl.enableStream(new StreamInfo(StreamType.FISH_EYE, 640, 480, FrameRate.FPS_30, PixelFormat.A8));*/
        }
    }

    @OnClick(R.id.button1)
    void btnA() {
        Algo.getInstance().funcA();
    }

    @OnClick(R.id.button2)
    void btnB() {
        Algo.getInstance().funcB();
    }

    @OnClick(R.id.button3)
    void btnC() {
        Algo.getInstance().funcC();
    }


    @OnClick(R.id.button4)
    public void startStopAlgo() {
        if (!mBind && !mIsSim) {
            return;
        }

        if (!mAlgoStarted) {
            startAlgo();
        } else {
            stopAlgo();
        }
    }

    @OnClick(R.id.button5)
    public void StartStopRecord() {
        if (checkSimulationStarted()) {
            return;
        }

        if (!mBind || !mAlgoStarted) {
            return;
        }

        // TODO: your code here
        if (mRecordStarted == false) {
            mRecordStarted = true;
            mAlgoRecordButton.setText("stop record");
            Algo.getInstance().startRecording(mRecordTmp);
        } else {
            mRecordStarted = false;
            mAlgoRecordButton.setText("start record");
            Algo.getInstance().stopRecording();
        }
    }

    @OnClick(R.id.button6)
    public void LocOfRecord() {
        // TODO: your code here
        if (mRecordTmp == false) {
            mRecordTmp = true;
            mRecordNewLoc.setText("Tmp");
        } else {
            mRecordTmp = false;
            mRecordNewLoc.setText("Date");
        }
    }

    @OnClick(R.id.button7)
    public void switchLiveSim() {
        if (mAlgoStarted)
            return;

        if (mIsSim == false) {
            mIsSim = true;
            mLiveSimSwitch.setText("Simu");
        } else {
            mIsSim = false;
            mLiveSimSwitch.setText("Live");
        }
    }

    private void startAlgo() {
        if (mAlgoStarted == true)
            return;

        mAlgoStarted = true;
        mAlgoTestButton.setText("stop algo");
        Algo.getInstance().start(mIsSim);
        if (mUseOpenglView) {
            renderStart();
        }
        if (myTimer != null) {
            myTimer.cancel();
        } else {
            myTimer = new Timer();
        }
        myTimer.scheduleAtFixedRate(new RenderTask(), 100, 50);

        if (!mIsSim) {
            VisionProxy.getInstance().startStreamIMUData();
            LocomotionProxy.getInstance().startStreamRobotInfo(Utils.isUseQuerySensorDataInterfaceOnG1(this));
            if (Utils.getRobotModel() == 1 && mBindExternalSuccess)
                LocomotionProxy.getInstance().startStreamRobotInfoExternal();
        }
    }

    private void stopAlgo() {
        Log.i(TAG, "stopAlgo: enter");
        if (mAlgoStarted == false)
            return;

        mAlgoStarted = false;
        mAlgoTestButton.setText("start algo");

        Log.i(TAG, "stopAlgo: stopRecording");
        if (mRecordStarted == true) {
            mRecordStarted = false;
            mAlgoRecordButton.setText("start record");
            Algo.getInstance().stopRecording();
        }

        Log.i(TAG, "stopAlgo: stopStreamIMUData,stopStreamRobotInfo");
        if (!mIsSim) {
            VisionProxy.getInstance().stopStreamIMUData();
            LocomotionProxy.getInstance().stopStreamRobotInfo();
            if (Utils.getRobotModel() == 1)
                LocomotionProxy.getInstance().stopStreamRobotInfoExternal();
        }

        Log.i(TAG, "stopAlgo: renderStop");
        if (mUseOpenglView) {
            renderStop();
        }

        Log.i(TAG, "stopAlgo: stop");
        if (myTimer != null) {
            myTimer.cancel();
            myTimer = null;
        }
        Algo.getInstance().stop();
    }

    public void renderStart() {
        mRelativeLayout = (RelativeLayout) findViewById(R.id.rel);
        RelativeLayout.LayoutParams mLayoutParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT, RelativeLayout.LayoutParams.MATCH_PARENT);
        mLayoutParams.addRule(RelativeLayout.BELOW, R.id.text0);
        mGlSurfaceView = new GLSurfaceView(MainActivity.this);
        mGlSurfaceView.setEGLContextClientVersion(2);
        mGlSurfaceView.setRenderer(mRenderer);
        Log.i(TAG, "runOnUiThread StartStopAlgo: addView");
        mGlSurfaceView.setLayoutParams(mLayoutParams);
        mRelativeLayout.addView(mGlSurfaceView);
    }

    public void renderStop() {

        mGlSurfaceView.onPause();
        mRelativeLayout.removeView(mGlSurfaceView);
    }

    private Timer myTimer;
    private Timer mLocoTimer;
    private static SimpleRunnable mDisplayRunnable;
    private static final String TAG = "AlgoTest";
    private ImageView mAlgoView;
    private TextView mTextView0;


    private class SimpleRunnable implements Runnable {
        private ImageView mView;
        private Bitmap mBitmap;
        private String mDebugInfo;

        SimpleRunnable(ImageView colorView) {
            mView = colorView;
        }

        public synchronized void setBitmaps(Bitmap color) {
            mBitmap = color;
        }

        public void setDebugInfo(String debugInfo) {
            this.mDebugInfo = debugInfo;
        }

        @Override
        public void run() {
            synchronized (this) {
                if (!mUseOpenglView) {
                    if (mDrawingRect != null) {
                        Canvas canvas = new Canvas(mBitmap);
                        Paint paint = new Paint();
                        paint.setColor(Color.GREEN);
                        paint.setStrokeWidth(5);
                        paint.setStyle(Paint.Style.STROKE);
                        canvas.drawRect(mDrawingRect, paint);
                    }
                    mView.setImageBitmap(mBitmap);
                }
                mTextView0.setText(mDebugInfo);
            }
        }
    }

    private class RenderTask extends TimerTask {
        @Override
        public void run() {
            synchronized (this) {
                appToShow();
            }
        }
    }

    public void appToShow() {
        //mDisplayRunnable.setDebugInfo(Algo.getInstance().getDebugInfo() + ", T::"+ Algo.getInstance().native_readSysfs(thermalpath));
        mDisplayRunnable.setDebugInfo(Algo.getInstance().getDebugInfo() + ", gitSHA:" + BuildConfig.GIT_SHA);
        if (!mUseOpenglView) {
            Algo.getInstance().renderBitmapByNative(AlgoBitmap);
            mDisplayRunnable.setBitmaps(AlgoBitmap);
        }
        runOnUiThread(mDisplayRunnable);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mTextView0 = (TextView) findViewById(R.id.text0);
        mAlgoView = (ImageView) findViewById(R.id.image1);

        mUseOpenglView = Algo.getInstance().isOpenGLView();
        mCameraConfig = Algo.getInstance().CameraConfigureType();

        mRenderer = new Renderer();
        AlgoBitmap = Bitmap.createBitmap(640, 360, Bitmap.Config.ARGB_8888);
        mDisplayRunnable = new SimpleRunnable(mAlgoView);
        Display display = getWindowManager().getDefaultDisplay();
        display.getSize(mScreenSize);
        Timber.plant(new Timber.DebugTree());
        ButterKnife.bind(this);

        VisionProxy.getInstance().setImpl(mVisionImpl);
        LocomotionProxy.getInstance().setHeadImpl(mRobotHeadImpl);
        LocomotionProxy.getInstance().setBaseImpl(mRobotBaseImpl);
        LocomotionProxy.getInstance().setSensorImpl(mRobotSensorImpl);
        LocomotionProxy.getInstance().setExternalImpl(mRobotExternalImpl);

        /*if (mIsSim) {
            mBindSwitch.setChecked(false);
            mBindSwitch.setClickable(false);
        }*/

        mEventReceiver = new EventReceiver(this, mEventListener);

        Algo.getInstance().init(this);

        mAlgoView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                Pair<Float, Float> start = null;
                if (v.getTag() != null) {
                    start = (Pair<Float, Float>) v.getTag();
                }
                float x = event.getAxisValue(MotionEvent.AXIS_X);
                float y = event.getAxisValue(MotionEvent.AXIS_Y);

//                Timber.d("action: " + event.getAction() + " x:" + x + " y:" + y);

                float viewLeft = 83f / 2;
                float viewTop = 0f / 2;
                float factor = 2.18f / 2;

//                Timber.d("view: x:" + viewLeft + " y:" + viewTop);

                int left;
                int top;
                int right;
                int bottom;

                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        start = new Pair<Float, Float>(x, y);
                        v.setTag(start);
                        left = regularX((int) ((x - viewLeft) * factor));
                        top = regularY((int) ((y - viewTop) * factor));
                        mDrawingRect = new Rect(left, top, left, top);
                        //Timber.d("rect:" + mDrawingRect);
                        break;
                    case MotionEvent.ACTION_UP:
                        // TODO: 16/6/8 notify new rect
                        //Timber.d("rect:" + mDrawingRect);
                        Algo.getInstance().newRect(mDrawingRect);
                        mDrawingRect = null;
                        break;
                    case MotionEvent.ACTION_MOVE:
                        left = regularX((int) ((start.first - viewLeft) * factor));
                        top = regularY((int) ((start.second - viewTop) * factor));
                        right = regularX((int) ((x - viewLeft) * factor));
                        bottom = regularY((int) ((y - viewTop) * factor));
                        if (left > right) {
                            int b = left;
                            left = right;
                            right = b;
                        }
                        if (top > bottom) {
                            int b = top;
                            top = bottom;
                            bottom = b;
                        }
                        mDrawingRect = new Rect(left, top, right, bottom);
                        //Timber.d("rect:" + mDrawingRect);
                        break;
                }

                return true;
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        stopAlgo();

        mVisionImpl.unbindService();
        mRobotHeadImpl.unbindService();
        mRobotBaseImpl.unbindService();
        mRobotSensorImpl.unbindService();
        if (mBindExternalSuccess)
            mRobotExternalImpl.unbindService();

        VisionProxy.getInstance().setImpl(null);
        LocomotionProxy.getInstance().setHeadImpl(null);
        LocomotionProxy.getInstance().setBaseImpl(null);
        LocomotionProxy.getInstance().setSensorImpl(null);
        LocomotionProxy.getInstance().setExternalImpl(null);
        mBind = false;
        WifiApManager.getInstance().disconnect();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // Pass the touch event to the native layer for camera control.
        // Single touch to rotate the camera around the device.
        // Two fingers to zoom in and out.
        int pointCount = event.getPointerCount();
        if (pointCount == 1) {
            float normalizedX = event.getX(0) / mScreenSize.x;
            float normalizedY = event.getY(0) / mScreenSize.y;
            Algo.getInstance().onTouchEventToNative(1, event.getActionMasked(),
                    normalizedX, normalizedY, 0.0f, 0.0f);
        }
        if (pointCount == 2) {
            if (event.getActionMasked() == MotionEvent.ACTION_POINTER_UP) {
                int index = event.getActionIndex() == 0 ? 1 : 0;
                float normalizedX = event.getX(index) / mScreenSize.x;
                float normalizedY = event.getY(index) / mScreenSize.y;
                Algo.getInstance().onTouchEventToNative(1, MotionEvent.ACTION_DOWN,
                        normalizedX, normalizedY, 0.0f, 0.0f);
            } else {
                float normalizedX0 = event.getX(0) / mScreenSize.x;
                float normalizedY0 = event.getY(0) / mScreenSize.y;
                float normalizedX1 = event.getX(1) / mScreenSize.x;
                float normalizedY1 = event.getY(1) / mScreenSize.y;
                Algo.getInstance().onTouchEventToNative(2, event.getActionMasked(),
                        normalizedX0, normalizedY0,
                        normalizedX1, normalizedY1);
            }
        }
        return true;
    }

    private int regularX(int x) {
        if (x < 0) return 0;
        if (x > 640) return 640;
        return x;
    }

    private int regularY(int y) {
        if (y < 0) return 0;
        if (y > 360) return 360;
        return y;
    }

    private boolean checkSimulationStarted() {
        if (mIsSim == true) {
            Timber.d("This is a Simulation Mode");
            Toast.makeText(MainActivity.this, "Simulation Mode do not support to record raw data", Toast.LENGTH_SHORT).show();
        }
        return mIsSim;
    }

    /*
    private void startVisionSample() {
        if (myTimer != null) {
            myTimer.cancel();
        } else {
            myTimer = new Timer();
        }
        myTimer.scheduleAtFixedRate(new SampleTask(), 0, 34);
    }

    private void stopVisionSample() {
        if (myTimer != null) {
            myTimer.cancel();
            myTimer = null;
        }
    }

    private void startControlSample() {
        if (mLocoTimer != null) {
            mLocoTimer.cancel();
        } else {
            mLocoTimer = new Timer();
        }
        mLocoTimer.scheduleAtFixedRate(new LocoMotionSampleTask(), 500, 500);
    }

    private void stopControlSample() {
        if (mLocoTimer != null) {
            mLocoTimer.cancel();
            mLocoTimer = null;
        }
    }

    private class LocoMotionSampleTask extends TimerTask {
        private class Show {
            String method;
            float value;

            public Show(String method, float value) {
                this.method = method;
                this.value = value;
            }

            @Override
            public String toString() {
                return "set " + method + " to " + value;
            }
        }

        long prevTime = System.currentTimeMillis();
        long SHOW_GAP = 2000;
        int parse;
        Show[] showSteps = {
                new Show("setWorldPitch", 0f),
                new Show("setWorldPitch", 1f),
                new Show("setWorldYaw", 1f),
                new Show("setWorldYaw", -1f),
                new Show("setWorldYaw", 0f),
                new Show("setIncrementalYaw", 1f),
                new Show("setIncrementalYaw", -1f),
                new Show("setJointPitch", 0f),
                new Show("setJointPitch", 1f),
                new Show("setJointYaw", 1f),
                new Show("setJointYaw", 0f),
                new Show("setLinearVelocity", 0.25f),
                new Show("setLinearVelocity", -0.25f),
                new Show("setLinearVelocity", 0f),
                new Show("setAngularVelocity", 0.1f),
                new Show("setAngularVelocity", -0.1f),
                new Show("setAngularVelocity", 0f),
        };

        @Override
        public void run() {
            // locomotion test
            /*if (System.currentTimeMillis() - prevTime > SHOW_GAP) {
                prevTime = System.currentTimeMillis();
                final Show show = showSteps[parse];
                parse = (parse + 1) % showSteps.length;
                try {
                    Method method = ControlManager.class.getMethod(show.method, float.class);
                    method.invoke(ControlManager.getInstance(MainActivity.this), show.value);
                    Log.d(TAG, "locomotion demo: " + show.toString());
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Toast.makeText(MainActivity.this, show.toString(), Toast.LENGTH_SHORT).show();
                        }
                    });
                } catch (NoSuchMethodException e) {
                    Log.e(TAG, "run: cannot find method", e);
                } catch (InvocationTargetException e) {
                    Log.e(TAG, "run: ", e);
                } catch (IllegalAccessException e) {
                    Log.e(TAG, "run: ", e);
                }
            }
        }
    }

    private class SampleTask extends TimerTask {
        final Bitmap colorBitmap = Bitmap.createBitmap(Resolution.getWidth(Resolution.Color.R_640_480),
                Resolution.getHeight(Resolution.Color.R_640_480), Bitmap.Config.ARGB_8888);
        final Bitmap depthBitmap = Bitmap.createBitmap(Resolution.getWidth(Resolution.Depth.R_320_240),
                Resolution.getHeight(Resolution.Depth.R_320_240), Bitmap.Config.RGB_565);
        final Bitmap fishEyeBitmap = Bitmap.createBitmap(Resolution.getWidth(Resolution.Color.R_640_480),
                Resolution.getHeight(Resolution.Color.R_640_480), Bitmap.Config.ALPHA_8);

        final Bitmap platformBitmap = Bitmap.createBitmap(PLATFORM_WIDTH, PLATFORM_HEIGHT, Bitmap.Config.RGB_565);

        @Override
        public void run() {
            Frame frame = VisionProxy.getInstance().getLatestColorFrame();
            if (frame != null) {
                colorBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                VisionProxy.getInstance().returnColorFrame(frame);
            } else {
                Timber.d("color frame not get");
            }

            frame = VisionProxy.getInstance().getLatestDepthFrame();
            if (frame != null) {
                depthBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                VisionProxy.getInstance().returnDepthFrame(frame);
            } else {
                Timber.d("depth frame not get");
            }

            frame = VisionProxy.getInstance().getLatestFishEyeFrame();
            if (frame != null) {
                fishEyeBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                VisionProxy.getInstance().returnFishEyeFrame(frame);
            } else {
                Timber.d("fish eye frame not get");
            }

            frame = VisionProxy.getInstance().getLatestPlatformFrame();
            if (frame != null) {
                ByteBuffer buf = ByteBuffer.allocate(PLATFORM_WIDTH * PLATFORM_HEIGHT * 2);
                buf.put(frame.getByteBuffer());
                buf.rewind();
                platformBitmap.copyPixelsFromBuffer(buf);
                VisionProxy.getInstance().returnPlatformFrame(frame);
            }


            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mColorImage.setImageBitmap(colorBitmap);
                    mDepthImage.setImageBitmap(depthBitmap);
                    mFishEyeImage.setImageBitmap(fishEyeBitmap);
                    mPlatformImage.setImageBitmap(platformBitmap);
                }
            });
        }
    }*/

    /**
     * for test use
     *
     * @return
     */
    protected boolean isBind() {
        return mBind;
    }
}
