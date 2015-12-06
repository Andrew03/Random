package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import com.qualcomm.ftcrobotcontroller.local.lib.LiftPIDThread;
import com.qualcomm.ftcrobotcontroller.local.lib.DrivePIDThread;
import com.qualcomm.ftcrobotcontroller.local.lib.TurnPIDThread;

/**
 * Created by Andrew on 11/20/2015.
 */
public class Test extends OpMode {
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

    // sensor declarations
    ColorSensor colorSensor         = null; // color sensor

    // all of the important constants
    final double    STOP                   = 0.0d,
                    MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER    = 0.65d,
                    LIFT_POWER      = 1.0d,
                    CLAMP_POWER     = 0.5d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = 0.05d,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = 0.05d,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_LIFT_START_POS_R                  = Servo.MIN_POSITION,
                    S_LIFT_START_POS_L                  = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_START_POS           = 0.37d,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
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
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

    // servo powers
    final double    S_BASKET_TILT_SPEED_LEFT    = 0.75d,
                    S_BASKET_TILT_SPEED_RIGHT   = 0.25d,
                    S_BASKET_TILT_SPEED_DOWN    = 0.55d,
                    S_BASKET_TILT_SPEED_STOP    = 0.5d;

    // servo speeds
    double S_basketTiltSpeed = S_BASKET_TILT_SPEED_STOP;

    // PID Threads
    LiftPIDThread liftPIDThread;
    DrivePIDThread drivePIDThread;
    TurnPIDThread turnPIDThread;

    // PID Threads Constants
    final double    liftKP  = 0.0d,
                    liftKI  = 0.0d,
                    liftKD  = 0.0d;
    final double    driveKP = 0.0d,
                    driveKD = 0.0d,
                    driveKI = 0.0d;
    final double    turnKP  = 0.003d, // orig 0.007 for 60, 0.0045 for 40
                    turnKI  = 0,//0.000025d,
                    turnKD  = 0.0d;

    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4
    } Quadrant quadrant;

    private final float C_STICK_TOP_THRESHOLD = 0.85f;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }
    private boolean isRed() {   return colorSensor.red() > colorSensor.blue();  }
    private boolean isBlue() {  return colorSensor.blue() > colorSensor.red();  }
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

        // mapping sensors to their hardware counterparts
        this.colorSensor            = this.hardwareMap.colorSensor.get("S_colorSensor");

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

        // defining PID Threads
        liftPIDThread = new LiftPIDThread(liftKP, liftKI, liftKD, this.M_lift);
        drivePIDThread = new DrivePIDThread(driveKP, driveKI, driveKD, this.M_driveFR, this.M_driveFL, this.M_driveBR, this.M_driveBL);
        turnPIDThread = new TurnPIDThread(turnKP, turnKI, turnKD, this.M_driveFR, this.M_driveFL, this.M_driveBR, this.M_driveBL);

        // starting PID Threads
        //liftPIDThread.start();
        //drivePIDThread.start();
        //turnPIDThread.start();
        //turnPIDThread.setTarget(910.0d);

    }

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
                    // use TurnPID variables, make an if statement



                    M_drivePowerR = turnPIDVal;
                    M_drivePowerL = turnPIDVal;
                } else {
                    M_drivePowerR = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                    M_drivePowerL = M_drivePowerR;
                }
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
            S_basketTiltSpeed = S_BASKET_TILT_SPEED_RIGHT;
        } else if(gamepad2.dpad_down) {
            S_basketTiltSpeed = S_BASKET_TILT_SPEED_LEFT;
        }

        if(gamepad2.dpad_right && S_basketRotatePos > 0.01d) {
            S_basketRotatePos -= 0.01d;
        } else if(gamepad2.dpad_left && S_basketRotatePos < 0.99d) {
            S_basketRotatePos += 0.01d;
        }
        if(gamepad2.start) {
            S_basketTiltSpeed = S_BASKET_TILT_SPEED_DOWN;
            S_basketRotatePos = S_BASKET_ROTATE_START_POS;
        }

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
        this.S_basketTilt.setPosition(this.S_basketTiltSpeed);
        //this.S_pickupSR.setPosition(this.S_pickupPosSR);
        //this.S_pickupSL.setPosition(this.S_pickupPosSL);
        //this.S_hitchR.setPosition(this.S_hitchPosR);
        //this.S_hitchL.setPosition(this.S_hitchPosL);

        //S_test.setPosition(S_testPos);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Servo Rot Pos", S_basketRotate.getPosition());
        telemetry.addData("Servo Tilt Pos", S_basketTilt.getPosition());
        if(isRed()) {
            telemetry.addData("It's red!", ":D");
        } else if(isBlue()) {
            telemetry.addData("It's blue!", ":(");
        }
    }
    @Override
    public void stop() {
        //turnPIDThread.interrupt();

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
}
