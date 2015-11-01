package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.ftcrobotcontroller.local.lib.Drive;
import com.qualcomm.ftcrobotcontroller.local.lib.PID;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 10/31/2015.
 */
public class Auton extends LinearOpMode {

    DcMotor M_driveFR, // front right drive motor
            M_driveFL, // front left drive motor
            M_driveBR, // back right drive motor
            M_driveBL, // back left drive motor
            M_pickup;  // pickup motor
    float   driveRPower = 0.0f,
            driveLPower = 0.0f;
    float distanceTravelledR;
    final double RIGHT_GEAR_RATIO = 0.0f;
    final double RIGHT_WHEEL_CIRCUMFERENCE = 0.0f;
    DriveThread R_driveThread;
    Thread T_driveThread;


    @Override
    public void runOpMode() {
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_pickup = hardwareMap.dcMotor.get("M_pickup");

        R_driveThread = new DriveThread(3, 5);
        T_driveThread = new Thread(R_driveThread);
        T_driveThread.start();

        R_driveThread.setTarget(12.0f);

        T_driveThread.interrupt();
    }

    private class DriveThread extends PID {
        public DriveThread(int gearRatio, int objectCircumference) {
            GEAR_RATIO = gearRatio;
            OBJECT_CIRCUMFERENCE = objectCircumference;
            kP = 0.0f;
            kI = 0.0f;
            kD = 0.0f;
            maxPower = 1.0f;
            minPower = -1.0f;
            minPIDPower = 0.2f;
            acceptableError = 50;
        }
        public void setTarget(float target) {
            this.target = target;
        }
    }

    private class DriveThreadRight implements Runnable {

         float  kP = 0.0f,
                kI = 0.0f,
                kD = 0.0f;
         float  maxPower = 1.0f,
                minPower = -1.0f,
                minPIDPower = 0.2f;
         int    acceptableError = 50;  // in encoder ticks
         float  target;           // in inches
         boolean isMoving = true,
                isFineTune = false;
         double  timer = 0.0f,   // in milliseconds
                fineTuneTimer = 0.0f,
                currDt = 0.0f;
         float   currError = 0.0f,   // in encoder ticks
                prevError = 0.0f,   // in encoder ticks
                errorRate = 0.0f,   // in encoder ticks
                accumError = 0.0f;  // in encoder ticks
         float   PIDValue = 0.0f,
                power = 0.0f;

         final int PULSE_PER_REV = 1120; // encoder ticks per revolution
         final double GEAR_RATIO = RIGHT_GEAR_RATIO,
                        OBJECT_CIRCUMFERENCE = RIGHT_WHEEL_CIRCUMFERENCE; // in inches
         ElapsedTime clock;

        @Override
        public void run() {
            clock.reset();
            clock.startTime();
            try {
                while(isMoving) {
                    currDt = clock.time() / 1000.0d;
                    clock.reset();
                    prevError = currError;
                    currError = target - distanceTravelledR;
                    accumError += currError;
                    errorRate = (prevError - currError) / (float)currDt;
                    PIDValue = kP * currError + kI * accumError + kD * errorRate;
                    PIDValue = Range.clip(PIDValue, -maxPower, maxPower);
                    driveRPower = PIDValue;
                    if(currError < acceptableError) {
                        power = 0.0f;
                        isMoving = false;
                    }
                    Thread.sleep(10);
                    // add in fine tune mode later
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }
}
