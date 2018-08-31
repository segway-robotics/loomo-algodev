package com.segway.robot.algo.algo_app;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

/**
 * Created by ark338 on 2016/11/17.
 * Receive top touch event
 */
public class EventReceiver extends BroadcastReceiver {
    // TODO: 2016/11/17 these two action should be get from sdk
    public static final String ACTION_TOP_TOUCH_DOWN = "com.segway.robot.action.TOP_TOUCH_DOWN";
    public static final String ACTION_TOP_TOUCH_UP = "com.segway.robot.action.TOP_TOUCH_UP";

    public static final int EVENT_HEAD_TOP_TOUCH_UP = 0;
    public static final int EVENT_HEAD_TOP_TOUCH_DOWN = 1;
    private final EventListener mEventListener;

    EventReceiver(Context context, EventListener eventListener) {
        if (context == null || eventListener == null) {
            throw new IllegalArgumentException("Arg is null");
        }
        mEventListener = eventListener;
        IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(ACTION_TOP_TOUCH_DOWN);
        intentFilter.addAction(ACTION_TOP_TOUCH_UP);
        context.registerReceiver(this, intentFilter);
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        String action = intent.getAction();
        switch (action) {
            case ACTION_TOP_TOUCH_DOWN:
                mEventListener.onNewEvent(EVENT_HEAD_TOP_TOUCH_DOWN);
                break;
            case ACTION_TOP_TOUCH_UP:
                mEventListener.onNewEvent(EVENT_HEAD_TOP_TOUCH_UP);
                break;
        }
    }

    interface EventListener {
        void onNewEvent(int event);
    }
}
