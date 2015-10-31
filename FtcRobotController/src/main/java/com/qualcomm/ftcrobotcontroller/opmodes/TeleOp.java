package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 10/23/2015.
 */

// the main teleop opmode we will be using
public class TeleOp extends OpMode {

    public TeleOp() {

    }
    DcMotor M_driveFR, // front right drive motor
            M_driveFL, // front left drive motor
            M_driveBR, // back right drive motor
            M_driveBL, // back left drive motor
            M_pickup;  // pickup motor
    float   driveRPower = 0.0f,
            driveLPower = 0.0f,
            pickupPower = 0.0f;
    boolean isPickup = false,
            isPickupReversed = false;
    ControllerThread R_controllerThread;
    Thread T_controllerThread;

    @Override
    public void init() {
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_pickup = hardwareMap.dcMotor.get("M_pickup");
        R_controllerThread = new ControllerThread();
        T_controllerThread = new Thread(R_controllerThread);

    }

    @Override
    public void start() {
        //controller.startThread();
        T_controllerThread.start();
    }

    @Override
    public void loop() {
        //driveRPower = controller.C1_stickRy;
        //driveLPower = controller.C1_stickLy;
        M_driveFR.setPower(driveRPower);
        M_driveFL.setPower(driveLPower);
        M_driveBR.setPower(driveRPower);
        M_driveBL.setPower(driveLPower);
        if(Math.abs(pickupPower) > 0.0f) {
            isPickup = true;
        } else {
            isPickup = false;
        }
        M_pickup.setPower(pickupPower);
        telemetry.addData("Text", "*** Robot Data***");

        telemetry.addData("R Power", "R Power: " + String.format("%.2f", driveRPower));
        telemetry.addData("L Power", "R Power: " + String.format("%.2f", driveLPower));
    }

    @Override
    public void stop() {
        T_controllerThread.interrupt();
    }

    private class ControllerThread implements Runnable {
        private final float C_STICK_TOP_THRESHOLD = 0.85f,      // least value for which stick value read from motor will be 1.0f
                C_STICK_BOTTOM_THRESHOLD = 0.05f,   // greatest value for which stick value read from motor will be 0.0f
                PICKUP_POWER = 0.9f;

        ControllerThread() {
            gamepad1.setJoystickDeadzone(C_STICK_BOTTOM_THRESHOLD);
            gamepad2.setJoystickDeadzone(C_STICK_BOTTOM_THRESHOLD);
        }

        // converts all of the controller sticks into more sensitive values
        // use a negative value for y axis since controller reads -1 when pushed forward
        private float convertStick(float controllerValue) {   return (float) Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

        // the main loop function
        public void run() {
            try {
                while(!Thread.currentThread().isInterrupted()) {
                    driveRPower = convertStick(-gamepad1.right_stick_y);
                    driveLPower = convertStick(-gamepad1.left_stick_y);
                    if(!isPickup)
                        if(gamepad1.right_bumper) {
                            pickupPower = PICKUP_POWER;
                        } else if(gamepad1.left_bumper) {
                            pickupPower = -PICKUP_POWER;
                        } else {
                            pickupPower = 0.0f;
                        }
                    else {
                        if(gamepad1.right_bumper || gamepad1.left_bumper) {
                            pickupPower = 0.0f;
                        }
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

