package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Andrew on 10/23/2015.
 */

// instructions: //////////////////////////
    /*
    left and right joysticksd gamepad 1 control driving, up down, tank drive
    right bumper to pick up, left bumper to push out from pickup
    right trigger to raise lift, left trigger to lower
    a to release balls, b to reset basket drop

    gamepad 2
    axby buttons control weird basket angles
    up on dpad will reset basket to original position

    fix on the robot
        lift doesn't go down all the way
        basket doesn't have a thing to stop balls from falling out
        guards on bottom to prevent balls from going in
        gaurds to prevent balls from going around isnide basket
        adhesive doesn't stick
        I don't have any decent auton so use nullOpMode if they make you select one
        run teleopmode for teleop
     */

// 1120 pulses per rev

// the main teleop opmode we will be using
public class TeleOp extends OpMode {

    public TeleOp() {

    }
    // motor declarations
    DcMotor M_driveFR   = null, // front right drive motor
            M_driveFL   = null, // front left drive motor
            M_driveBR   = null, // back right drive motor
            M_driveBL   = null, // back left drive motor
            M_pickup    = null, // pickup motor
            M_lift      = null, // lift motor
            M_clamp     = null, // right hang motor
            M_hangL     = null; // left hang motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_liftR                 = null, // right servo that supports lift
            S_liftL                 = null, // left servo that supports lift
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_basketTilt            = null, // front left servo of the pickup
            S_pickupSR              = null, // servo on right side of the pickup
            S_pickupSL              = null, // servo on left side of the pickup
            S_hitchR                = null, // right hitch servo
            S_hitchL                = null; // left hitch servo
    // all of the important constants
    final double    STOP                   = 0.0d,
                    MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER    = 0.8d,
                    LIFT_POWER      = 1.0d,
                    CLAMP_POWER     = 0.5d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = 0.05d,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = 0.05d,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_LIFT_START_POS_R                  = Servo.MIN_POSITION,
                    S_LIFT_START_POS_L                  = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_START_POS           = 0.61d,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
                    S_BASKET_TILT_START_POS             = 0.61d,
                    S_PICKUP_START_POS_SR               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SL               = Servo.MIN_POSITION,
                    S_HITCH_START_POS_R                 = Servo.MIN_POSITION,
                    S_HITCH_START_POS_L                 = Servo.MIN_POSITION;

    // all of the ending/close servo positions
    final double    S_CLIMBERS__KNOCKDOWN_END_POS_R     = 0.22d,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = 0.22d,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_LIFT_END_POS_R                    = Servo.MAX_POSITION,
                    S_LIFT_END_POS_L                    = Servo.MAX_POSITION,
                    S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_BASKET_TILT_END_POS               = Servo.MIN_POSITION,
                    S_PICKUP_END_POS_FL                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SR                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SL                 = Servo.MAX_POSITION,
                    S_HITCH_END_POS_R                   = Servo.MAX_POSITION,
                    S_HITCH_END_POS_L                   = Servo.MAX_POSITION;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_clampPower  = STOP,
            M_hangPowerL  = STOP;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_liftPosR               = S_LIFT_START_POS_R,
            S_liftPosL               = S_LIFT_START_POS_L,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_basketTiltPos          = S_BASKET_TILT_START_POS,
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

    enum DriveModes {
        TANK,   // the traditional y axis stick values driving method for ftc
        FPS,    // driver controls angle and direction from that angle
        ARCADE, // driver controls solely direction, angle only when robot at rest
        THRUST  // driver controls angle and power
    }
    DriveModes driveMode;

    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4,
    } Quadrant quadrantL;

    private Quadrant findQuadrant(double stickY, double stickX) {
        if(stickY >= 0) {
            if(stickX >= 0) {
                return Quadrant.Q1;
            } else {
                return Quadrant.Q2;
            }
        } else {
            if(stickX >= 0) {
                return Quadrant.Q3;
            } else {
                return Quadrant.Q4;
            }
        }
    }

    // finds target in degrees
    private double findTarget(double stickY, double stickX, Quadrant quadrant) {
        switch(quadrant) {
            case Q1:
            case Q4:
                return 90 - Math.toDegrees(Math.asin(-stickY / stickX));
            default:
                return -90 - Math.toDegrees(Math.asin(-stickY / stickX));
        }
    }

    private final float C_STICK_TOP_THRESHOLD = 0.85f;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }


    @Override
    public void init() {
        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");
        //this.M_clamp    = this.hardwareMap.dcMotor.get("M_clamp");
        //this.M_hangL    = this.hardwareMap.dcMotor.get("M_hangL");

        // mapping servo variables to their hardware counterparts
        this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        //this.S_liftR                = this.hardwareMap.servo.get("S_liftR");
        //this.S_liftL                = this.hardwareMap.servo.get("S_liftL");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");
        //this.S_pickupSR             = this.hardwareMap.servo.get("S_pickupSR");
        //this.S_pickupSL             = this.hardwareMap.servo.get("S_pickupSL");
        //this.S_hitchR               = this.hardwareMap.servo.get("S_hitchR");
        //this.S_hitchL               = this.hardwareMap.servo.get("S_hitchL");

        // fixing motor directions
        this.M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        this.M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        this.M_pickup.setDirection(DcMotor.Direction.REVERSE);
        this.M_lift.setDirection(DcMotor.Direction.REVERSE);
        //this.M_hangL.setDirection(DcMotor.Direction.REVERSE);

        // resets all the encoder values
        this.M_driveFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_lift.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_hangL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // this.M_hangR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        this.M_driveFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_lift.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    @Override
    public void start() {

    }

    //double ticksToDegrees;
    @Override
    public void loop() {
        /*double encoderValueRs = (M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2;
        double encoderValueLs = (M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2;
        double currentAngle = (encoderValueLs - encoderValueRs) / 2 * ticksToDegrees;
        double targetAngle;
        quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        switch(driveMode) {
            case TANK:
                M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                M_drivePowerL = convertStick(-gamepad1.left_stick_y);
                break;
            case FPS:
                break;
            case ARCADE:
                double turnThreshold = 0.25d;
                quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                currentAngle = (encoderValueLs - encoderValueRs) / 2 * ticksToDegrees;  // in degrees
                targetAngle = findTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, quadrantL); // in degrees
                if(Math.abs(targetAngle - currentAngle) > turnThreshold) {
                    
                }
                M_drivePowerR = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                M_drivePowerL = M_drivePowerR;
                break;
            case THRUST:
                quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                M_drivePowerR = gamepad1.right_stick_y;
                M_drivePowerL = M_drivePowerR;
                break;
            default:
                break;
        }
        */
        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);

        if (gamepad1.right_bumper) {
            M_pickupPower = PICKUP_POWER;
        } else if (gamepad1.left_bumper) {
            M_pickupPower = -PICKUP_POWER;
        } else {
            M_pickupPower = STOP;
        }

        // lift control block
        if(gamepad1.right_trigger > 0.0f) {
            M_liftPower = LIFT_POWER;
        } else if(gamepad1.left_trigger > 0.0f) {
            M_liftPower = -LIFT_POWER;
        } else {
            M_liftPower = STOP;
        }

        // basket control block
        if(gamepad1.a) {
            S_basketReleasePos = S_BASKET_RELEASE_END_POS;
        } else if(gamepad1.b) {
            S_basketReleasePos = S_BASKET_RELEASE_START_POS;
        }

        // climber deposit pos
        if(gamepad1.y && S_climbersDepositPos > 0.04) {
            S_climbersDepositPos -= 0.03d;
        } else if(gamepad1.x) {
            S_climbersDepositPos = S_CLIMBERS_DEPOSIT_START_POS;
        }

        if(gamepad2.dpad_up) {
            if(S_basketTiltPos < 0.99d) {
                S_basketTiltPos += 0.01d;
            }
        } else if(gamepad2.dpad_down) {
            if(S_basketTiltPos > 0.01d) {
                S_basketTiltPos -= 0.01d;
            }
        }

        if(gamepad2.dpad_right && S_basketRotatePos > 0.01d) {
            S_basketRotatePos -= 0.01d;
        } else if(gamepad2.dpad_left && S_basketRotatePos < 0.99d) {
            S_basketRotatePos += 0.01d;
        }
        if(gamepad2.back) {
            S_basketTiltPos = S_BASKET_TILT_START_POS;
            S_basketRotatePos = S_BASKET_ROTATE_START_POS;
        }
        if(gamepad2.back) {
            S_basketTiltPos = S_BASKET_TILT_START_POS;
            S_basketRotatePos = S_BASKET_ROTATE_START_POS;
        }
        /*if(gamepad2.right_trigger > 0.0d) {
            if(S_climbersKnockdownPosR < 0.99d) {
                S_climbersKnockdownPosR += 0.01d;
            }
        } else if(gamepad2.left_trigger > 0.0d) {
            if(S_climbersKnockdownPosR > 0.01d) {
                S_climbersKnockdownPosR -= 0.01d;
            }
        }

        if(gamepad2.right_bumper) {
            if(S_climbersKnockdownPosL < 0.99d) {
                S_climbersKnockdownPosL += 0.01d;
            }
        } else if(gamepad2.left_bumper) {
            if(S_climbersKnockdownPosL > 0.01d) {
                S_climbersKnockdownPosL -= 0.01d;
            }
        }*/

        if(gamepad2.a) {
            S_climbersKnockdownPosR = S_CLIMBERS__KNOCKDOWN_END_POS_R;
            S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
        } else if(gamepad2.b) {
            S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_START_POS_R;
            S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_START_POS_L;
        }

        // updates all the motor powers
        this.M_driveBR.setPower(this.M_drivePowerR);
        this.M_driveBL.setPower(this.M_drivePowerL);
        this.M_driveFR.setPower(this.M_drivePowerR);
        this.M_driveFL.setPower(this.M_drivePowerL);
        this.M_pickup.setPower(this.M_pickupPower);
        this.M_lift.setPower(this.M_liftPower);
        //this.M_clamp.setPower(this.M_clampPower);
        //this.M_hangL.setPower(this.M_hangPowerL);

        // updates all the servo positions
        this.S_climbersKnockdownR.setPosition(this.S_climbersKnockdownPosR);
        this.S_climbersKnockdownL.setPosition(this.S_climbersKnockdownPosL);
        this.S_climbersDeposit.setPosition(this.S_climbersDepositPos);
        //this.S_liftR.setPosition(this.S_liftPosR);
        //this.S_liftL.setPosition(this.S_liftPosL);
        this.S_basketRotate.setPosition(this.S_basketRotatePos);
        this.S_basketRelease.setPosition(this.S_basketReleasePos);
        this.S_basketTilt.setPosition(this.S_basketTiltPos);
        //this.S_pickupSR.setPosition(this.S_pickupPosSR);
        //this.S_pickupSL.setPosition(this.S_pickupPosSL);
        //this.S_hitchR.setPosition(this.S_hitchPosR);
        //this.S_hitchL.setPosition(this.S_hitchPosL);


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Servo Rot Pos", S_basketRotate.getPosition());
        telemetry.addData("Servo Tilt Pos", S_basketTilt.getPosition());
        telemetry.addData("Climber R Pos", S_climbersKnockdownR.getPosition());
        telemetry.addData("Climber L Pos", S_climbersKnockdownL.getPosition());

    }

    @Override
    public void stop() {
        this.M_driveBR.setPower(STOP);
        this.M_driveBL.setPower(STOP);
        this.M_driveFR.setPower(STOP);
        this.M_driveFL.setPower(STOP);
        this.M_pickup.setPower(STOP);
        this.M_lift.setPower(STOP);
        //this.M_clamp.setPower(STOP);
        //this.M_hangL.setPower(STOP);

        this.S_climbersKnockdownR.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_R);
        this.S_climbersKnockdownL.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_L);
        this.S_climbersDeposit.setPosition(S_CLIMBERS_DEPOSIT_START_POS);
        //this.S_liftR.setPosition(S_LIFT_START_POS_R);
        //this.S_liftL.setPosition(S_LIFT_START_POS_L);
        this.S_basketRotate.setPosition(S_BASKET_ROTATE_START_POS);
        this.S_basketRelease.setPosition(S_BASKET_RELEASE_START_POS);
        //this.S_pickupFL.setPosition(this.S_pickupPosFL);
        //this.S_pickupSR.setPosition(this.S_pickupPosSR);
        //this.S_pickupSL.setPosition(this.S_pickupPosSL);
        //this.S_hitchR.setPosition(this.S_hitchPosR);
        //this.S_hitchL.setPosition(this.S_hitchPosL);
    }

    /*private class ControllerThread implements Runnable {
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

    }*/
}

