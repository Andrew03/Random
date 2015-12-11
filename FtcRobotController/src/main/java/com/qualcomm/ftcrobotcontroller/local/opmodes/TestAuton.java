package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/**
 * Created by Andrew on 12/11/2015.
 */
public class TestAuton extends OpMode {

    // motor declarations
    DcMotor M_driveFR   = null, // front right drive motor
            M_driveFL   = null, // front left drive motor
            M_driveBR   = null, // back right drive motor
            M_driveBL   = null, // back left drive motor
            M_pickup    = null, // pickup motor
            M_lift      = null; // lift motor

    // servo declarations
    Servo S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_basketTilt            = null, // front left servo of the pickup
            S_buttonPusher          = null;
    // all of the important constants
    final double    STOP                   = 0.0d,
            MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER    = 0.8d,
            LIFT_POWER      = 1.0d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
            S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MAX_POSITION,
            S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
            S_BASKET_ROTATE_START_POS           = 0.37d,
            S_BASKET_TILT_START_POS             = 0.875d,
            S_BASKET_RELEASE_START_POS          = 0.34d,
            S_BUTTON_PUSHER_START_POS           = Servo.MIN_POSITION;


    // all of the ending/close servo positions
    final double    S_CLIMBERS_KNOCKDOWN_END_POS_R      = 0.494d,
            S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MIN_POSITION,
            S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
            S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
            S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
            S_BUTTON_PUSHER_END_POS             = 0.141d;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;
    double[] drivePowers;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_basketTiltPos          = S_BASKET_TILT_START_POS,
            S_buttonPusherPos        = S_BUTTON_PUSHER_START_POS;

    private void mapStuff() {
        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");

        // mapping servo variables to their hardware counterparts
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");

    }

    private void configureStuff() {
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
    public void init() {
        mapStuff();
        configureStuff();
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        if(gamepad1.b) {
            M_drivePowerR = -0.3d;
            M_drivePowerL = 0.3d;
        } else if(gamepad1.x) {
            M_drivePowerR = 0.3d;
            M_drivePowerL = -0.3d;
        } else if(gamepad1.a) {
            M_drivePowerR = -0.3d;
            M_drivePowerL = -0.3d;
        } else if(gamepad1.y) {
            M_drivePowerR = 0.3d;
            M_drivePowerL = 0.3d;
        } else {
            M_drivePowerR = 0.0d;
            M_drivePowerL = 0.0d;
        }

        M_driveFR.setPower(M_drivePowerR);
        M_driveFL.setPower(M_drivePowerL);
        M_driveBR.setPower(M_drivePowerR);
        M_driveBL.setPower(M_drivePowerL);

        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
    }
    @Override
    public void stop() {

    }

}
