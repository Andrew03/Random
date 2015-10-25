package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Andrew on 10/24/2015.
 */

public class Auton extends OpMode{
    // all of the motor declarations
    DcMotor M_driveFR, // front right drive motor
            M_driveFL, // front left drive motor
            M_driveBR, // back right drive motor
            M_driveBL, // back left drive motor
            M_pickup,  // pickup motor
            M_lift,    // lift motor
            M_hangR,   // right hanging motor
            M_hangL;   // left hanging motor
    // all of the servo declarations
    Servo   S_climbersR, // right servo that knocks down climbers
            S_climbersL, // left servo that knocks down climbers
            S_liftR,     // right servo that supports lift
            S_liftL,     // left servo that supports lift
            S_basketR,   // right servo on the basket
            S_basketL,   // left servo on the basket
            S_pickupFR,  // front right servo of the pickup
            S_pickupFL,  // front left servo of the pickup
            S_pickupSR,  // servo on right side of the pickup
            S_pickupSL,  // servo on left side of the pickup
            S_hitchR,    // right hitch servo
            S_hitchL;    // left hitch servo

    // all of the motor declarations
    volatile float  driveRPower,    // right drive motor's power value
            driveLPower,    // left drive motor's power value
            liftPower;
    volatile boolean    isDrive = false,    // tells if robot is currently driving
            isTurn = false,     // tells if robot is currently turning
            isLift = false;     // tells if robot is currently lifting
    final float pickupPower = 1.0f;     // pickup motor's power value

    @Override
    public void init() {
        // all of the motor definitions
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_pickup = hardwareMap.dcMotor.get("M_pickup");
        M_lift = hardwareMap.dcMotor.get("M_lift");
        M_hangR = hardwareMap.dcMotor.get("M_hangR");
        M_hangL = hardwareMap.dcMotor.get("M_hangL");
        // all of the servo definitions
        S_climbersR = hardwareMap.servo.get("S_climbersR");
        S_climbersL = hardwareMap.servo.get("S_climbersL");
        S_liftR = hardwareMap.servo.get("S_liftR");
        S_liftL = hardwareMap.servo.get("S_liftL");
        S_basketR = hardwareMap.servo.get("S_basketR");
        S_basketL = hardwareMap.servo.get("S_basketL");
        S_pickupFR = hardwareMap.servo.get("S_pickupFR");
        S_pickupFL = hardwareMap.servo.get("S_pickupFL");
        S_pickupSR = hardwareMap.servo.get("S_pickupSR");
        S_pickupSL = hardwareMap.servo.get("S_pickupSL");
        S_hitchR = hardwareMap.servo.get("S_hitchR");
        S_hitchL = hardwareMap.servo.get("S_hitchL");
        // instances of all of the threads
        DrivePID_Thread drivePID_loop = new DrivePID_Thread();
        TurnPID_Thread turnPID_loop = new TurnPID_Thread();
        LiftPID_Thread liftPID_Thread = new LiftPID_Thread();
    }


    @Override
    public void loop() {

    }

    // drive PID thread
    private class DrivePID_Thread extends Thread {
        volatile  float driveRPower,
                driveLPower;
        final float P = 0.0f,
                I = 0.0f,
                D = 0.0f;
        public void run() {
            try {
                while (!isInterrupted()) {

                    sleep(100);
                }
            }
            catch (InterruptedException e) {}
            catch (Throwable e) {}
        }
    }
    // turn PID thread
    private class TurnPID_Thread extends Thread {
        volatile  float driveRPower,
                driveLPower;
        final float P = 0.0f,
                I = 0.0f,
                D = 0.0f;
        public void run() {
            try {
                while (!isInterrupted()) {

                    sleep(100);
                }
            }
            catch (InterruptedException e) {}
            catch (Throwable e) {}
        }
    }
    // lift PID thread
    private class LiftPID_Thread extends Thread {
        volatile  float liftPower;
        final float P = 0.0f,
                I = 0.0f,
                D = 0.0f;
        public void run() {
            try {
                while (!isInterrupted()) {

                    sleep(100);
                }
            }
            catch (InterruptedException e) {}
            catch (Throwable e) {}
        }
    }

}
