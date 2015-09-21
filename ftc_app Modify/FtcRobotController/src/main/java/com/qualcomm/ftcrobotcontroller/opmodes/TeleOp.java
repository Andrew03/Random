package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftcrobotcontroller.opmodes.AccelerometerSensor;

/**
 * Created by Andrew on 9/12/2015.
 */
public class TeleOp extends OpMode {

    Context mContext;
    public TeleOp(Context mContext) {
        this.mContext = mContext;
    }

    static DcMotor m_leftFrontDrive;
    static DcMotor m_rightFrontDrive;
    static DcMotor m_leftBackDrive;
    static DcMotor m_rightBackDrive;
    static GyroSensor g_gyro;
    private SensorManager p_SensorManager;
    private GyroSensor p_gyro;
    private AccelerometerSensor a_accel;

    @Override
    public void init() {

        // setting all the hardware
        m_leftBackDrive = hardwareMap.dcMotor.get("leftFrontDrive");
        m_rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive");
        m_leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");
        m_rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        // setting the phones gyroscope
        p_SensorManager = (SensorManager)mContext.getSystemService(Context.SENSOR_SERVICE);
        p_gyro = new GyroSensor(p_SensorManager);
        a_accel = new AccelerometerSensor(p_SensorManager);

        // reversing right side drive motors
        m_rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        m_rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        float g1_leftY = -gamepad1.left_stick_y;
        float g1_leftX = gamepad1.left_stick_x;
        float g1_rightY = -gamepad1.right_stick_y;
        float g1_rightX = gamepad1.right_stick_x;
        ElapsedTime T; // variable that keeps track of time
        T = new ElapsedTime();
        double tTime = T.time();

        // encoders count 1440 ticks per rotation
        // 1440 ticks = pi * d
        // try runToPosition function for PID
        //Range.clip(val, -1, 1);
        // square values to make more sensitive in the middle
    }

}
