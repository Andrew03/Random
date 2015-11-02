package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 10/23/2015.
 */

// 1120 pulses per rev

// the main teleop opmode we will be using
public class TeleOp extends OpMode {

    public TeleOp() {

    }
    DcMotor M_driveFR, // front right drive motor
            M_driveFL, // front left drive motor
            M_driveBR, // back right drive motor
            M_driveBL, // back left drive motor
            M_pickup,  // pickup motor
            M_lift;    // lift motor
            //M_hangR,   // right hanging motor
            //M_hangL;   // left hanging motor
    // all of the servo declarations
    /*Servo   S_climbersR, // right servo that knocks down climbers
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
            */
    float   driveRPower = 0.0f,
            driveLPower = 0.0f,
            pickupPower = 0.0f,
            liftPower   = 0.0f;
    boolean isPickup = false;
    ControllerThread R_controllerThread;
    Thread T_controllerThread;

    final float STOP = 0.0f;
    // all of the possible drive modes
    enum DriveModes {
        TANK,   // the traditional y axis stick values driving method for ftc
        FPS,    // driver controls angle and direction from that angle
        ARCADE, // driver controls solely direction, angle only when robot at rest
        THRUST  // driver controls angle and power
    }
    DriveModes driveMode;


    @Override
    public void init() {
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_pickup = hardwareMap.dcMotor.get("M_pickup");
        M_lift = hardwareMap.dcMotor.get("M_lift");
        /*M_hangR = hardwareMap.dcMotor.get("M_hangR");
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
        S_hitchL = hardwareMap.servo.get("S_hitchL");*/

        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        M_pickup.setDirection(DcMotor.Direction.REVERSE);
        M_lift.setDirection(DcMotor.Direction.REVERSE);

        R_controllerThread = new ControllerThread();
        T_controllerThread = new Thread(R_controllerThread, "R_controllerThread");
        // other definitions
        driveMode = DriveModes.TANK;

    }

    @Override
    public void start() {
        T_controllerThread.start();
    }

    @Override
    public void loop() {
        M_driveFR.setPower(driveRPower);
        M_driveFL.setPower(driveLPower);
        M_driveBR.setPower(driveRPower);
        M_driveBL.setPower(driveLPower);
        M_pickup.setPower(pickupPower);
        M_lift.setPower(liftPower);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("R Power", "R Power: " + String.format("%.2f", driveRPower));
        telemetry.addData("L Power", "R Power: " + String.format("%.2f", driveLPower));
    }

    @Override
    public void stop() {
        T_controllerThread.interrupt();
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
        M_pickup.setPower(STOP);
        M_lift.setPower(STOP);
    }

    private class ControllerThread implements Runnable {
        private final float C_STICK_TOP_THRESHOLD = 0.85f,      // least value for which stick value read from motor will be 1.0f
                            PICKUP_POWER = 1.0f,
                            LIFT_POWER = 1.0f;

        // converts all of the controller sticks into more sensitive values
        // use a negative value for y axis since controller reads -1 when pushed forward
        private float convertStick(float controllerValue) {   return (float) Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

        // the main loop function
        public void run() {
            try {
                while(!Thread.currentThread().isInterrupted()) {
                    driveRPower = convertStick(-gamepad1.right_stick_y);
                    driveLPower = convertStick(-gamepad1.left_stick_y);

                    if (gamepad1.right_bumper) {
                        pickupPower = PICKUP_POWER;
                    } else if (gamepad1.left_bumper) {
                        pickupPower = -PICKUP_POWER;
                    } else {
                        pickupPower = STOP;
                    }

                    if(gamepad1.right_trigger > 0.0f) {
                        liftPower = LIFT_POWER;
                    } else if(gamepad1.left_trigger > 0.0f) {
                        liftPower = -LIFT_POWER;
                    } else {
                        liftPower = STOP;
                    }

                    telemetry.addData("Thread is running", "Thread is running");
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

    }
}

