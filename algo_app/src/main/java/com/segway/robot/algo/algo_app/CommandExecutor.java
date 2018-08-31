package com.segway.robot.algo.algo_app;

import android.util.Log;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * Created by admin on 2017/6/20.
 */

public class CommandExecutor {
    private static final String TAG = "CommandExecutor";

    public static String exeCmd(String[] cmd) {
        Runtime rt = Runtime.getRuntime();
        Process proc = null;
        try {
            File wd = new File("/system/bin/");
            String[] env = {
                    "HOME=/data",
                    "ANDROID_ASSETS=/system/app",
                    "ANDROID_DATA=/data",
                    "ANDROID_PROPERTY_WORKSPACE=9,0",
                    "ANDROID_ROOT=/system",
                    "ANDROID_SOCKET_adbd=11",
                    "ANDROID_STORAGE=/storage",
                    "HOSTNAME=VA50EC_1",
                    "MKSH=/system/bin/sh",
                    "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin",
                    "SHELL=/system/bin/sh",
                    "USER=shell"
            };

            proc = rt.exec(cmd, env); //,wd
            BufferedReader stdInput = new BufferedReader(new
                    InputStreamReader(proc.getInputStream()));

            BufferedReader stdError = new BufferedReader(new
                    InputStreamReader(proc.getErrorStream()));

            String s = null;
            String s_out = "";
            while ((s = stdInput.readLine()) != null) {
                Log.d(TAG, "exeCmd stdout:" + s);
                s_out = s_out+s;
            }

            while ((s = stdError.readLine()) != null) {
                Log.d(TAG, "exeCmd stderr:" + s);
            }
            return s_out;
        } catch (IOException e) {
            Log.e(TAG, "exeCmd: ", e);
            return "";
        }
    }

    public static String exeCmd(String cmd) {
        Runtime rt = Runtime.getRuntime();
        Process proc = null;
        try {
            File wd = new File("/system/bin/");
            String[] env = {
                    "HOME=/data",
                    "ANDROID_ASSETS=/system/app",
                    "ANDROID_DATA=/data",
                    "ANDROID_PROPERTY_WORKSPACE=9,0",
                    "ANDROID_ROOT=/system",
                    "ANDROID_SOCKET_adbd=11",
                    "ANDROID_STORAGE=/storage",
                    "HOSTNAME=VA50EC_1",
                    "MKSH=/system/bin/sh",
                    "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin",
                    "SHELL=/system/bin/sh",
                    "USER=shell"
            };

            proc = rt.exec(cmd, env); //,wd
            BufferedReader stdInput = new BufferedReader(new
                    InputStreamReader(proc.getInputStream()));

            BufferedReader stdError = new BufferedReader(new
                    InputStreamReader(proc.getErrorStream()));

            String s = null;
            String s_out = "";
            while ((s = stdInput.readLine()) != null) {
                Log.d(TAG, "exeCmd stdout:" + s);
                s_out = s_out+s;
            }

            while ((s = stdError.readLine()) != null) {
                Log.d(TAG, "exeCmd stderr:" + s);
            }
            return s_out;
        } catch (IOException e) {
            Log.e(TAG, "exeCmd: ", e);
            return "";
        }
    }
}
