/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.bluetoothlegatt;

import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import java.util.List;
import java.util.UUID;

/**
 * Service for managing connection and data communication with a GATT server hosted on a
 * given Bluetooth LE device.
 */
public class BluetoothLeService extends Service {
    private final static String TAG = BluetoothLeService.class.getSimpleName();

    private BluetoothManager mBluetoothManager;
    private BluetoothAdapter mBluetoothAdapter;
    private String mBluetoothDeviceAddress;
    private BluetoothGatt mBluetoothGatt;
    private int mConnectionState = STATE_DISCONNECTED;

    private static final int STATE_DISCONNECTED = 0;
    private static final int STATE_CONNECTING = 1;
    private static final int STATE_CONNECTED = 2;

    public final static String ACTION_GATT_CONNECTED =
            "com.example.bluetooth.le.ACTION_GATT_CONNECTED";
    public final static String ACTION_GATT_DISCONNECTED =
            "com.example.bluetooth.le.ACTION_GATT_DISCONNECTED";
    public final static String ACTION_GATT_SERVICES_DISCOVERED =
            "com.example.bluetooth.le.ACTION_GATT_SERVICES_DISCOVERED";
    public final static String ACTION_DATA_AVAILABLE =
            "com.example.bluetooth.le.ACTION_DATA_AVAILABLE";
    public final static String EXTRA_DATA =
            "com.example.bluetooth.le.EXTRA_DATA";

    public final static UUID UUID_HEART_RATE_MEASUREMENT =
            UUID.fromString(SampleGattAttributes.HEART_RATE_MEASUREMENT);
    public final static UUID UUID_AIR_MEASUREMENT =
            UUID.fromString(SampleGattAttributes.Air_Pressure_MEASUREMENT );
    public final static UUID UUID_AIR_COMPENSATION1 =
            UUID.fromString(SampleGattAttributes.Air_Pressure_COMPENSATION1);
    public final static UUID UUID_AIR_COMPENSATION2 =
            UUID.fromString(SampleGattAttributes.Air_Pressure_COMPENSATION2);

    static  int bmp280RawPressure = 0;
    static  int bmp280RawTemperature = 0;



    // Implements callback methods for GATT events that the app cares about.  For example,
    // connection change and services discovered.
    private final BluetoothGattCallback mGattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            String intentAction;
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                intentAction = ACTION_GATT_CONNECTED;
                mConnectionState = STATE_CONNECTED;
                broadcastUpdate(intentAction);
                Log.i(TAG, "Connected to GATT server.");
                // Attempts to discover services after successful connection.
                Log.i(TAG, "Attempting to start service discovery:" +
                        mBluetoothGatt.discoverServices());

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                intentAction = ACTION_GATT_DISCONNECTED;
                mConnectionState = STATE_DISCONNECTED;
                Log.i(TAG, "Disconnected from GATT server.");
                broadcastUpdate(intentAction);
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                broadcastUpdate(ACTION_GATT_SERVICES_DISCOVERED);
            } else {
                Log.w(TAG, "onServicesDiscovered received: " + status);
            }
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);
        }
    };

    private void broadcastUpdate(final String action) {
        final Intent intent = new Intent(action);
        sendBroadcast(intent);
    }
    private double t;
    private double p;
    int bytetoint(byte s) {
        return ((int)s & 0x00000000000000FF);
    }
    private void broadcastUpdate(final String action,
                                 final BluetoothGattCharacteristic characteristic) {
        final Intent intent = new Intent(action);
        UUID uuid = characteristic.getUuid();

        // This is special handling for the Heart Rate Measurement profile.  Data parsing is
        // carried out as per profile specifications:
        // http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml
        // if (UUID_HEART_RATE_MEASUREMENT.equals(characteristic.getUuid())) {
        if (uuid.equals(UUID_AIR_MEASUREMENT )) {

            int bmp280RawPressure = 0;
            int bmp280RawTemperature = 0;
            final byte[] data = characteristic.getValue();
            bmp280RawPressure= (((bytetoint(data[0])) << 12) | ((bytetoint(data[1])) << 4) | (bytetoint(data[2]) >> 4));
            bmp280RawTemperature=(((bytetoint(data[3])) << 12) | ((bytetoint(data[4])) << 4) | (bytetoint(data[5]) >> 4));

            t = getUnsignedInt(bmp280CompensateT(bmp280RawTemperature))/100.0;
            p = getUnsignedInt(bmp280CompensateP(bmp280RawPressure))/25600.0;
            intent.putExtra(EXTRA_DATA,"pressure:" +  Double.toString(p)+"\n"+"temperature:"+Double.toString(t)+"\n");
        } else if (uuid.equals(UUID_AIR_COMPENSATION1 )) {
            final byte[] data = characteristic.getValue();
            bmp280Cal.dig_T1=take_short(data[0],data[1]);
            bmp280Cal.dig_T2=take_short(data[2],data[3]);
            bmp280Cal.dig_T3=take_short(data[4],data[5]);
            bmp280Cal.dig_P1=take_short(data[6],data[7]);
            bmp280Cal.dig_P2=take_short(data[8],data[9]);
            bmp280Cal.dig_P3=take_short(data[10],data[11]);
            bmp280Cal.dig_P4=take_short(data[12],data[13]);
            bmp280Cal.dig_P5=take_short(data[14],data[15]);
            bmp280Cal.dig_P6=take_short(data[16],data[17]);
            bmp280Cal.dig_P7=take_short(data[18],data[19]);
            if (data != null && data.length > 0) {
                final StringBuilder stringBuilder = new StringBuilder(data.length);
                for (byte byteChar : data)
                    stringBuilder.append(String.format("%02X ", byteChar));
                intent.putExtra(EXTRA_DATA, stringBuilder.toString());
            }

        }
        else if(uuid.equals(UUID_AIR_COMPENSATION2 ))
        {
            final byte[] data = characteristic.getValue();
            bmp280Cal.dig_P8=take_short(data[0],data[1]);//采集是对的
            bmp280Cal.dig_P9=take_short(data[2],data[3]);
            bmp280Cal.t_fine=take_int(data[4],data[5],data[6],data[7]);
            if (data != null && data.length > 0) {
                final StringBuilder stringBuilder = new StringBuilder(data.length);
                for (byte byteChar : data)
                    stringBuilder.append(String.format("%02X ", byteChar));
                intent.putExtra(EXTRA_DATA,  stringBuilder.toString());
            }
        }
        else {
            // For all other profiles, writes the data formatted in HEX.
            final byte[] data = characteristic.getValue();
            if (data != null && data.length > 0) {
                final StringBuilder stringBuilder = new StringBuilder(data.length);
                for (byte byteChar : data)
                    stringBuilder.append(String.format("%02X ", byteChar));
                intent.putExtra(EXTRA_DATA, stringBuilder.toString());
            }
        }
        sendBroadcast(intent);
    }
    public long getUnsignedInt (int data){     //将int数据转换为0~4294967295 (0xFFFFFFFF即DWORD)。
        return data&0x0FFFFFFFFl;
    }
    short take_short(byte a,byte b)//a是低位，b是高位
    {
         short c=(short)(((short)b<<8)&0xFF00);
         c=(short)(c|((short)a&0x00FF));
       //  c=(short)(c|a);
         return c;
    }

    int take_int(byte a,byte b,byte c, byte d)
    {
        int e=(int)(((int)d)<<24) ;
        e=(int)(e| (((int)c)<<16));
        e=(int)(e|(((int)b)<<8));
        e=(int)(e|(int)a);
        return e;
    }

    int shorttoint(short s)
    {
        return ((int )s &0x0000FFFF);
    }

    // Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
    private bmp280Calib bmp280Cal=new bmp280Calib();
    private int bmp280CompensateT(int adcT)
    {
        int var1, var2, T;

        var1 = ((((adcT >> 3) - (shorttoint(bmp280Cal.dig_T1)<< 1))) * ((int)bmp280Cal.dig_T2)) >> 11;
        var2  = (((((adcT >> 4) - (shorttoint(bmp280Cal.dig_T1))) * ((adcT >> 4) - (shorttoint(bmp280Cal.dig_T1)))) >> 12) * ((int)bmp280Cal.dig_T3)) >> 14;
        bmp280Cal.t_fine = var1 + var2;

        T = (bmp280Cal.t_fine * 5 + 128) >> 8;

        return T;
    }
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
   // Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    long shorttolong(short s)
    {
        return ((long)s &0x000000000000FFFF);
    }
    int longtoint(long s)
    {
        return (int)(s&0x00000000FFFFFFFF);
    }
    long inttolong(int s)
    {
        return ((long)s &0x00000000FFFFFFFF);
    }
    int bmp280CompensateP(int adcP)
    {
//        long var1, var2, p;
//        var1 = (inttolong(bmp280Cal.t_fine)) - 128000L;
//        var2 = var1 * var1 * shorttolong(bmp280Cal.dig_P6);
//        var2 = var2 + ((var1*shorttolong(bmp280Cal.dig_P5)) << 17);
//        var2 = var2 + ((shorttolong(bmp280Cal.dig_P4)) << 35);
//        var1 = ((var1 * var1 * shorttolong(bmp280Cal.dig_P3)) >>> 8) + ((var1 * shorttolong(bmp280Cal.dig_P2)) << 12);
//        var1 = ((((0x0000000000000001) << 47) + var1)) * (shorttolong(bmp280Cal.dig_P1)) >>> 33;
//        if (var1 == 0)
//            return 0;
//        p = ((long)(1048576L - adcP))&0x00000000FFFFFFFF;
//        p = (((p << 31) - var2) * 3125L) / var1;
//        var1 = ((shorttolong(bmp280Cal.dig_P9)) * (p >>> 13) * (p >>> 13)) >>> 25;
//        var2 = ((shorttolong(bmp280Cal.dig_P8)) * p) >>> 19;
//        p = ((p + var1 + var2) >>> 8) + ((shorttolong(bmp280Cal.dig_P7)) << 4);
//        return longtoint(p);
        long var1, var2, p;
        var1 = ((long)bmp280Cal.t_fine) - 128000;
        var2 = var1 * var1 * (long)bmp280Cal.dig_P6;
        var2 = var2 + ((var1*(long)bmp280Cal.dig_P5) << 17);
        var2 = var2 + (((long)bmp280Cal.dig_P4) << 35);
        var1 = ((var1 * var1 * (long)bmp280Cal.dig_P3) >> 8) + ((var1 * (long)bmp280Cal.dig_P2) << 12);
        var1 = (((((long)1) << 47) + var1)) * ((long)shorttolong(bmp280Cal.dig_P1)) >> 33;
        if (var1 == 0)
            return 0;
        p = 1048576 - adcP;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((long)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((long)bmp280Cal.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((long)bmp280Cal.dig_P7) << 4);
        return (int)p;
    }


    private class bmp280Calib
    {
        short dig_T1;	/* calibration T1 data */
        short dig_T2; /* calibration T2 data */
        short dig_T3; /* calibration T3 data */
        short dig_P1;  /* calibration P1 data */
        short dig_P2; /* calibration P2 data */
        short dig_P3; /* calibration P3 data */
        short dig_P4; /* calibration P4 data */
        short dig_P5; /* calibration P5 data */
        short dig_P6; /* calibration P6 data */
        short dig_P7; /* calibration P7 data */
        short dig_P8; /* calibration P8 data */
        short dig_P9; /* calibration P9 data */
        int t_fine; /* calibration t_fine data */
    }//校准数据

    public class LocalBinder extends Binder {
        BluetoothLeService getService() {
            return BluetoothLeService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        // After using a given device, you should make sure that BluetoothGatt.close() is called
        // such that resources are cleaned up properly.  In this particular example, close() is
        // invoked when the UI is disconnected from the Service.
        close();
        return super.onUnbind(intent);
    }

    private final IBinder mBinder = new LocalBinder();

    /**
     * Initializes a reference to the local Bluetooth adapter.
     *
     * @return Return true if the initialization is successful.
     */
    public boolean initialize() {
        // For API level 18 and above, get a reference to BluetoothAdapter through
        // BluetoothManager.
        if (mBluetoothManager == null) {
            mBluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
            if (mBluetoothManager == null) {
                Log.e(TAG, "Unable to initialize BluetoothManager.");
                return false;
            }
        }

        mBluetoothAdapter = mBluetoothManager.getAdapter();
        if (mBluetoothAdapter == null) {
            Log.e(TAG, "Unable to obtain a BluetoothAdapter.");
            return false;
        }

        return true;
    }

    /**
     * Connects to the GATT server hosted on the Bluetooth LE device.
     *
     * @param address The device address of the destination device.
     *
     * @return Return true if the connection is initiated successfully. The connection result
     *         is reported asynchronously through the
     *         {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     *         callback.
     */
    public boolean connect(final String address) {
        if (mBluetoothAdapter == null || address == null) {
            Log.w(TAG, "BluetoothAdapter not initialized or unspecified address.");
            return false;
        }

        // Previously connected device.  Try to reconnect.
        if (mBluetoothDeviceAddress != null && address.equals(mBluetoothDeviceAddress)
                && mBluetoothGatt != null) {
            Log.d(TAG, "Trying to use an existing mBluetoothGatt for connection.");
            if (mBluetoothGatt.connect()) {
                mConnectionState = STATE_CONNECTING;
                return true;
            } else {
                return false;
            }
        }

        final BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
        if (device == null) {
            Log.w(TAG, "Device not found.  Unable to connect.");
            return false;
        }
        // We want to directly connect to the device, so we are setting the autoConnect
        // parameter to false.
        mBluetoothGatt = device.connectGatt(this, false, mGattCallback);
        Log.d(TAG, "Trying to create a new connection.");
        mBluetoothDeviceAddress = address;
        mConnectionState = STATE_CONNECTING;
        return true;
    }

    /**
     * Disconnects an existing connection or cancel a pending connection. The disconnection result
     * is reported asynchronously through the
     * {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     * callback.
     */
    public void disconnect() {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.disconnect();
    }

    /**
     * After using a given BLE device, the app must call this method to ensure resources are
     * released properly.
     */
    public void close() {
        if (mBluetoothGatt == null) {
            return;
        }
        mBluetoothGatt.close();
        mBluetoothGatt = null;
    }

    /**
     * Request a read on a given {@code BluetoothGattCharacteristic}. The read result is reported
     * asynchronously through the {@code BluetoothGattCallback#onCharacteristicRead(android.bluetooth.BluetoothGatt, android.bluetooth.BluetoothGattCharacteristic, int)}
     * callback.
     *
     * @param characteristic The characteristic to read from.
     */
    public void readCharacteristic(BluetoothGattCharacteristic characteristic) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.readCharacteristic(characteristic);
    }

    /**
     * Enables or disables notification on a give characteristic.
     *
     * @param characteristic Characteristic to act on.
     * @param enabled If true, enable notification.  False otherwise.
     */
    public void setCharacteristicNotification(BluetoothGattCharacteristic characteristic,
                                              boolean enabled) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.setCharacteristicNotification(characteristic, enabled);

        // This is specific to Heart Rate Measurement.
        if (UUID_HEART_RATE_MEASUREMENT.equals(characteristic.getUuid())) {
            BluetoothGattDescriptor descriptor = characteristic.getDescriptor(
                    UUID.fromString(SampleGattAttributes.CLIENT_CHARACTERISTIC_CONFIG));
            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            mBluetoothGatt.writeDescriptor(descriptor);
        }
    }

    /**
     * Retrieves a list of supported GATT services on the connected device. This should be
     * invoked only after {@code BluetoothGatt#discoverServices()} completes successfully.
     *
     * @return A {@code List} of supported services.
     */
    public List<BluetoothGattService> getSupportedGattServices() {
        if (mBluetoothGatt == null) return null;

        return mBluetoothGatt.getServices();
    }
}
