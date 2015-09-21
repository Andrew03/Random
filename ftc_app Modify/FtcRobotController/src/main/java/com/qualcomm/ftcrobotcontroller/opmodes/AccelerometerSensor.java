package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by Andrew on 9/19/2015.
 */
public class AccelerometerSensor implements SensorEventListener {
    private Sensor accelerometer;
    float[] val;
    AccelerometerSensor(SensorManager sensorManager) {
        val = new float[3];
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }


    public float[] getVal() {
        return val;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        val[0] = sensorEvent.values[0];
        val[1] = sensorEvent.values[1];
        val[2] = sensorEvent.values[2];
    }
}
