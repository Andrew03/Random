package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;


/**
 * Created by Andrew on 9/15/2015.
 */

public class GyroSensor implements SensorEventListener {

    private Sensor gyro;
    private float currentRot[];
    private float initOrientation[];
    // conversion from nannoseconds to seconds
    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;
    // should be very small, just over 0
    private final float EPSILON = 0.000001f;

    GyroSensor(SensorManager sensorManager, float[] gravity, float[] geomagnetic) {
        gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        currentRot = new float[3];
        sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_NORMAL);
        if(!calculateInitOrientation(gravity, geomagnetic)) {
            // log an error
        }
    }

    public float[] getCurrentRot() {
        return currentRot;
    }
    
    @Override
    public void onAccuracyChanged(Sensor Sensor, int accuracy) {

    }
    
    @Override
    public void onSensorChanged(SensorEvent event) {

        if(timestamp != 0) {
            // dT is change in time
            final float dT = (event.timestamp - timestamp) * NS2S;
            // the rotation values from the device
            float axisX = event.values[0];
            float axisY = event.values[1];
            float axisZ = event.values[2];

            // the magnitude of rotation vector
            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            // normalizing and finding unit vectors
            if (omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            // need unit quaternions for the getRotationMatrixFromVector method
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;

            float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;
        }
        timestamp = event.timestamp;
        float[] deltaRotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
        currentRot = multiplyMatrix(currentRot, deltaRotationMatrix);
    }

    private boolean calculateInitOrientation(float[] gravity, float[] geomagnetic) {
        return getRotationMatrix(currentRot, null, gravity, geomagnetic);
    }

    private float[] multiplyMatrix(float[] a, float[] b) {
        float[] result = new float[9];

        result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
        result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
        result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

        result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
        result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
        result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

        result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
        result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
        result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

        return result;
    }

}
