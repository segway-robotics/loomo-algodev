package com.segway.robot.algo.algo_app;

import android.content.Intent;
import android.test.ActivityTestCase;
import android.test.ActivityUnitTestCase;
import android.test.suitebuilder.annotation.LargeTest;
import android.test.suitebuilder.annotation.SmallTest;
import android.widget.Button;
import android.widget.TextView;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by cattani on 2017/1/9.
 */
public class MainActivityTest extends ActivityUnitTestCase<MainActivity>{
    MainActivity mainActivity;

    public MainActivityTest() {
        super(MainActivity.class);
    }

    @Override
    public void setUp() throws Exception {
        super.setUp();

        getInstrumentation().runOnMainSync(new Runnable() {
            @Override
            public void run() {
                // Starts the MainActivity of the target application
                startActivity(new Intent(getInstrumentation().getTargetContext(), MainActivity.class), null, null);

                // Getting a reference to the MainActivity of the target application
                mainActivity = (MainActivity)getActivity();
            }
        });
    }

    @Override
    public void tearDown() throws Exception {

    }

    @LargeTest
    public void testBind() throws Exception {
        final Button bindButton = (Button) mainActivity.findViewById(R.id.bind);
        final Button funcaButton = (Button) mainActivity.findViewById(R.id.button1);
        TextView debugInfo = (TextView) mainActivity.findViewById(R.id.text0);

        getInstrumentation().runOnMainSync(new Runnable() {
            @Override
            public void run() {
                bindButton.performClick();
            }
        });

        Thread.sleep(10000);

        String info = (String) debugInfo.getText();

        Assert.assertTrue(mainActivity.isBind());
        Assert.assertTrue(info.startsWith("APP_"));

        if(info.startsWith("APP_VIO"))
            return;

        /*getInstrumentation().runOnMainSync(new Runnable() {
            @Override
            public void run() {
                funcaButton.performClick();
            }
        });

        Thread.sleep(3000);*/

        getInstrumentation().runOnMainSync(new Runnable() {
            @Override
            public void run() {
                bindButton.performClick();
            }
        });

        Thread.sleep(5000);
    }

    @LargeTest
    public void testSuccess() throws Exception {

    }

}