package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
            M_hangR     = null, // right hang motor
            M_hangL     = null; // left hang motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_liftR                 = null, // right servo that supports lift
            S_liftL                 = null, // left servo that supports lift
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_pickupFL              = null, // front left servo of the pickup
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
                    LIFT_POWER      = 1.0d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_LIFT_START_POS_R                  = Servo.MIN_POSITION,
                    S_LIFT_START_POS_L                  = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_START_POS           = Servo.MIN_POSITION,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
                    S_PICKUP_START_POS_FL               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SR               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SL               = Servo.MIN_POSITION,
                    S_HITCH_START_POS_R                 = Servo.MIN_POSITION,
                    S_HITCH_START_POS_L                 = Servo.MIN_POSITION;

    // all of the ending/close servo positions
    final double    S_CLIMBERS__KNOCKDOWN_END_POS_R     = Servo.MAX_POSITION,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MAX_POSITION,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_LIFT_END_POS_R                    = Servo.MAX_POSITION,
                    S_LIFT_END_POS_L                    = Servo.MAX_POSITION,
                    S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_FR                 = Servo.MAX_POSITION,
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
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_liftPosR               = S_LIFT_START_POS_R,
            S_liftPosL               = S_LIFT_START_POS_L,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_pickupPosFL            = S_PICKUP_START_POS_FL,
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

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

    @Override
    public void init() {
        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");
        //this.M_hangR    = this.hardwareMap.dcMotor.get("M_hangR");
        //this.M_hangL    = this.hardwareMap.dcMotor.get("M_hangL");

        // mapping servo variables to their hardware counterparts
        //this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        //this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        //this.S_liftR                = this.hardwareMap.servo.get("S_liftR");
        //this.S_liftL                = this.hardwareMap.servo.get("S_liftL");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        //this.S_pickupFL             = this.hardwareMap.servo.get("S_pickupFL");
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
        //this.M_hangR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_hangL.setMode(DcMotorController.RunMode.RESET_ENCODERS);

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
        turnPIDThread.start();
        turnPIDThread.setTarget(910.0d);

    }

    @Override
    public void loop() {
        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);

        //this.M_driveFR.setPower(this.M_drivePowerR);
        //this.M_driveFL.setPower(this.M_drivePowerL);
        //this.M_driveBR.setPower(this.M_drivePowerR);
        //this.M_driveBL.setPower(this.M_drivePowerL);

        // pickup control block
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
        if(gamepad1.x) {
            S_basketReleasePos = S_BASKET_RELEASE_END_POS;
        } else if(gamepad1.y) {
            S_basketReleasePos = S_BASKET_RELEASE_START_POS;
        }
        /*
                if(gamepad1.a) {
                    S_basketRotatePos += 0.01d;
                } else if(gamepad1.b) {
                    S_basketRotatePos -= 0.01d;
                }
                */
        // climber knockdown control block
                /*
                if(gamepad1.a) {
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
                    S_climbersKnockdownPosR = S_CLIMBERS__KNOCKDOWN_END_POS_R;
                } else if(gamepad1.b) {
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_START_POS_L;
                    S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_START_POS_R;
                }
                */
                /*
                if(gamepad1.y) {
                    S_climbersDepositPos = S_CLIMBERS_DEPOSIT_END_POS;
                } else if(gamepad1.x) {
                    S_climbersDepositPos = S_CLIMBERS_DEPOSIT_START_POS;
                }
                */
        // updates all the motor powers
        //this.M_driveBR.setPower(this.M_drivePowerR);
        //this.M_driveBL.setPower(this.M_drivePowerL);
        //this.M_driveFR.setPower(this.M_drivePowerR);
        //this.M_driveFL.setPower(this.M_drivePowerL);
        this.M_pickup.setPower(this.M_pickupPower);
        this.M_lift.setPower(this.M_liftPower);
        //this.M_hangR.setPower(this.M_hangPowerR);
        //this.M_hangL.setPower(this.M_hangPowerL);

        // updates all the servo positions
        //this.S_climbersKnockdownR.setPosition(this.S_climbersKnockdownPosR);
        //this.S_climbersKnockdownL.setPosition(this.S_climbersKnockdownPosL);
        this.S_climbersDeposit.setPosition(this.S_climbersDepositPos);
        //this.S_liftR.setPosition(this.S_liftPosR);
        //this.S_liftL.setPosition(this.S_liftPosL);
        this.S_basketRotate.setPosition(this.S_basketRotatePos);
        this.S_basketRelease.setPosition(this.S_basketReleasePos);
        //this.S_pickupFL.setPosition(this.S_pickupPosFL);
        //this.S_pickupSR.setPosition(this.S_pickupPosSR);
        //this.S_pickupSL.setPosition(this.S_pickupPosSL);
        //this.S_hitchR.setPosition(this.S_hitchPosR);
        //this.S_hitchL.setPosition(this.S_hitchPosL);

        telemetry.addData("AVG Right Pos", (M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2);
        telemetry.addData("AVG Left Pos", (M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2);
        /*telemetry.addData("RF Motor Power", M_driveFR.getPower());
        telemetry.addData("RB Motor Power", M_driveBR.getPower());
        telemetry.addData("LF Motor Power", M_driveFL.getPower());
        telemetry.addData("LB Motor Power", M_driveBL.getPower());


        //telemetry.addData("RF Motor Supposed Power", turnPIDThread.getPowers()[0]);
        //telemetry.addData("LF Motor Supposed Power", turnPIDThread.getPowers()[1]);
        telemetry.addData("RB Motor Supposed Power", turnPIDThread.getPowers()[2]);
        telemetry.addData("LB Motor Supposed Power", turnPIDThread.getPowers()[3]);

        //telemetry.addData("RF PID VALUE", turnPIDThread.getPIDValue(0));
        //telemetry.addData("LF PID VALUE", turnPIDThread.getPIDValue(1));
        telemetry.addData("RB PID VALUE", turnPIDThread.getPIDValue(2));
        telemetry.addData("LB PID VALUE", turnPIDThread.getPIDValue(3));

        telemetry.addData("RF curr pos", turnPIDThread.getCurrPos(0));
        telemetry.addData("LF curr pos", turnPIDThread.getCurrPos(1));
        telemetry.addData("RB curr pos", turnPIDThread.getCurrPos(2));
        telemetry.addData("LB curr pos", turnPIDThread.getCurrPos(3));
        */
        //telemetry.addData("Lift Motor Pos", M_lift.getCurrentPosition());
    }
    @Override
    public void stop() {
        turnPIDThread.interrupt();

        this.M_driveBR.setPower(STOP);
        this.M_driveBL.setPower(STOP);
        this.M_driveFR.setPower(STOP);
        this.M_driveFL.setPower(STOP);
        this.M_pickup.setPower(STOP);
        this.M_lift.setPower(STOP);
        //this.M_hangR.setPower(STOP);
        //this.M_hangL.setPower(STOP);

        //this.S_climbersKnockdownR.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_R);
        //this.S_climbersKnockdownL.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_L);
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
