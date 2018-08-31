/*
 * Copyright 2013 WhiteByte (Nick Russler, Ahmet Yueksektepe).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.segway.robot.algo.algo_app.wifiap;

import android.content.Context;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.util.Log;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

public class WifiApManager {
    private static final String TAG = "WifiApManager";
    private WifiManager mWifiManager;
    private Context mContext;
    private static WifiApManager mWifiApManager = null;
    private WifiApStateListener mWifiApStateListener = null;
    WifiConfiguration mWifiConfiguration = null;

    public static synchronized WifiApManager getInstance() {
        if (mWifiApManager == null) {
            mWifiApManager = new WifiApManager();
        }
        return mWifiApManager;
    }

    private WifiApManager() {

    }

    public void initialize(Context context,WifiApStateListener listener) {
        mContext = context;
        mWifiManager = (WifiManager) mContext.getSystemService(Context.WIFI_SERVICE);
        mWifiApStateListener = listener;
        String robotName = Build.SERIAL;
        mWifiConfiguration = mWifiApManager.createWifiConfiguration("Algo-" + robotName, "12345678", WifiApUtil.WPA_PSK);
        mWifiApManager.setWifiApEnabled(mWifiConfiguration, true);
    }

    public void setWifiApEnabled(WifiConfiguration wifiConfig, boolean enabled) {
        mWifiManager.setWifiEnabled(false);
        try {
            Method method = mWifiManager.getClass().getMethod("setWifiApEnabled", WifiConfiguration.class, boolean.class);
            method.invoke(mWifiManager, wifiConfig, enabled);
        } catch (NoSuchMethodException e) {
            Log.e(TAG, "disconnect: ", e);
        } catch (InvocationTargetException e) {
            Log.e(TAG, "disconnect: ", e);
        } catch (IllegalAccessException e) {
            Log.e(TAG, "disconnect: ", e);
        }
    }

    public int getWifiApState() {
        try {
            Method method = mWifiManager.getClass().getMethod("getWifiApState");
            return ((Integer) method.invoke(mWifiManager));
        } catch (Exception e) {
            Log.e(this.getClass().toString(), "", e);
            return WifiApUtil.WIFI_AP_STATE_FAILED;
        }
    }

    public WifiConfiguration getWifiApConfiguration() {
        try {
            Method method = mWifiManager.getClass().getMethod("getWifiApConfiguration");
            return (WifiConfiguration) method.invoke(mWifiManager);
        } catch (Exception e) {
            Log.e(this.getClass().toString(), "", e);
            return null;
        }
    }

    public boolean setWifiApConfiguration(WifiConfiguration wifiConfig) {
        try {
            Method method = mWifiManager.getClass().getMethod("setWifiApConfiguration", WifiConfiguration.class);
            return (Boolean) method.invoke(mWifiManager, wifiConfig);
        } catch (Exception e) {
            Log.e(this.getClass().toString(), "", e);
            return false;
        }
    }

    public WifiConfiguration createWifiConfiguration(String SSID, String password, int type) {
        Log.d(TAG, "SSID = " + SSID + ";Password =" + password + ";Type = " + type);
        WifiConfiguration config = new WifiConfiguration();
        config.SSID = SSID;
        clearAll(SSID);
        switch (type) {
            case WifiApUtil.NONE:
                config.hiddenSSID = false;
                config.status = WifiConfiguration.Status.ENABLED;
                config.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.NONE);
                config.allowedProtocols.set(WifiConfiguration.Protocol.WPA);
                config.allowedProtocols.set(WifiConfiguration.Protocol.RSN);
                config.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.TKIP);
                config.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.CCMP);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP40);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP104);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.TKIP);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.CCMP);
                config.preSharedKey = null;
                break;
            case WifiApUtil.WPA_EAP:
                config.hiddenSSID = true;
                config.wepKeys[0] = password;
                config.allowedAuthAlgorithms.set(WifiConfiguration.AuthAlgorithm.SHARED);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.CCMP);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.TKIP);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP40);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP104);
                config.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.NONE);
                config.wepTxKeyIndex = 0;
                break;
            case WifiApUtil.WPA_PSK:
                config.preSharedKey = password;
                config.hiddenSSID = true;
                config.allowedAuthAlgorithms.set(WifiConfiguration.AuthAlgorithm.OPEN);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.TKIP);
                config.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.CCMP);
                config.allowedProtocols.set(WifiConfiguration.Protocol.WPA);
                config.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.CCMP);
                config.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.TKIP);
                config.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.WPA_PSK);
                config.status = WifiConfiguration.Status.ENABLED;
                break;
        }
        return config;
    }

    private void clearAll(String SSID) {
        List<WifiConfiguration> existingConfigs = mWifiManager.getConfiguredNetworks();
        if (existingConfigs == null) {
            return;
        }
        for (WifiConfiguration existingConfig : existingConfigs) {
            Log.d(TAG, "SSID=" + existingConfig.SSID + ",netID = " + existingConfig.networkId);
            if (existingConfig.SSID.equals("\"" + SSID + "\"")) {
                mWifiManager.disableNetwork(existingConfig.networkId);
                mWifiManager.removeNetwork(existingConfig.networkId);
            }
        }
        mWifiManager.saveConfiguration();
    }

    public void disconnect() {
        if(mWifiConfiguration != null)
            setWifiApEnabled(mWifiConfiguration, false);
    }
}
