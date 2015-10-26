package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Andrew on 10/23/2015.
 */

// the main teleop opmode we will be using
public class TeleOp extends OpMode {

    // all of the motor declarations
    DcMotor M_driveFR, // front right drive motor
            M_driveFL, // front left drive motor
            M_driveBR, // back right drive motor
            M_driveBL, // back left drive motor
            M_pickup;  // pickup motor
            //M_lift,    // lift motor
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
    // all of the possible drive modes
    enum DriveModes {
        TANK,   // the traditional y axis stick values driving method for ftc
        FPS,    // driver controls angle and direction from that angle
        ARCADE, // driver controls solely direction, angle only when robot at rest
        THRUST  // driver controls angle and power
    }
    DriveModes driveMode;

    // the controller object
    Controller controller;

    // all of the motor variable declarations & definitions
    float   driveLPower,
            driveRPower,
            pickupPower = 0.9f,
            liftUpPower = 0.9f,
            liftDownPower = -0.9f;
    boolean isPickup = false;

    // runs once before everything else
    @Override
    public void init() {

        // all of the motor definitions
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_pickup = hardwareMap.dcMotor.get("M_pickup");
        /*M_lift = hardwareMap.dcMotor.get("M_lift");
        M_hangR = hardwareMap.dcMotor.get("M_hangR");
        M_hangL = hardwareMap.dcMotor.get("M_hangL");*/

        // all of the servo definitions
        /*S_climbersR = hardwareMap.servo.get("S_climbersR");
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

        // other definitions
        controller = new Controller(gamepad1, gamepad2);
        driveMode = DriveModes.TANK;

        // start threads
        controller.startThread();   // the thread that finds all of the controller values
    }

    // the code that runs time and time again
    @Override
    public void loop() {

        // decides which drive mode robot is in
        if(controller.C1_dUp) {
            driveMode = DriveModes.TANK;    // up on dpad of controller 1 sets robot in tank mode
        } else if(controller.C1_dRight) {
            driveMode = DriveModes.FPS;     // right on dpad of controller 1 sets robot in FPS mode
        } else if(controller.C1_dDown) {
            driveMode = DriveModes.ARCADE;  // down on dpad of controller 1 sets robot in arcade mode
        } else if(controller.C1_dLeft) {
            driveMode = DriveModes.THRUST;  // left on dpad of controller 1 sets robot in thrust mode
        }

        // runs different blocks depending on which drive mode robot is in
        switch(driveMode) {
            // in tank mode
            case TANK:
                driveRPower = controller.C1_stickRy;
                driveLPower = controller.C1_stickLy;
                break;
            // in fps mode
            case FPS:
                break;
            // in arcade mode
            case ARCADE:
                break;
            // in thrust mode
            case THRUST:
                break;
            default:
                break;
        }

        /* pickup control block
         * toggle, press once to start and press again to deactivate
         */
        if(!isPickup) {
            if(gamepad1.right_bumper) {
                // runs pickup if controller 1's right bumper is pressed
                M_pickup.setPower(pickupPower);
                isPickup = true;
            } else if(gamepad2.left_bumper) {
                // runs pickup in reverse if controller 1's left bumper is pressed
                M_pickup.setPower(-pickupPower);
                isPickup = true;
            }
        } else if(gamepad1.right_bumper || gamepad1.left_bumper) {
            // stops pickup if either of controller 1's bumpers are pressed while pickup is running
            M_pickup.setPower(0.0f);
            isPickup = false;
        }


        // lift control block
        if(controller.isC1_triggerR) {
            // runs pickup if controller 1's right trigger is held
            //M_lift.setPower(liftUpPower);
        } else if(controller.isC1_triggerL) {
            // runs pickup in reverse if controller 1's left trigger is held
            //M_lift.setPower(liftDownPower);

        } else {
            // stops pickup if neither trigger is pressed
            M_pickup.setPower(0.0f);
        }

        driveRPower = gamepad1.right_stick_y;
        driveLPower = gamepad1.left_stick_y;

        // drive power block
        M_driveFR.setPower(driveRPower);
        M_driveFL.setPower(driveLPower);
        M_driveBR.setPower(driveRPower);
        M_driveBL.setPower(driveLPower);

        telemetry.addData("left", "left:  " + String.format("%.2f", controller.C1_stickLy));
        telemetry.addData("left", "left:  " + String.format("%.2f", driveRPower));
        telemetry.addData("right", "right:  " + String.format("%.2f", controller.C1_stickRy));
        telemetry.addData("right", "right:  " + String.format("%.2f", driveLPower));
    }
}
