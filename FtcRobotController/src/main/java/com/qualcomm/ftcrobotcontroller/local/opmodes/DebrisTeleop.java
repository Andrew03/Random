package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Anjali on 1/17/2016.
 */
  public class DebrisTeleop extends OpMode {

    DcMotor M_driveBR,
            M_driveBL,
            M_driveFR,
            M_driveFL,
            M_liftR,
            M_liftL,
            M_pickup,
            M_basket;

    Servo   S_basket,
            S_climberKnockdownR,
            S_climberKnockdownL,
            S_climberDepositDrop,
            S_climberDepositSwing;

    final double C_STICK_TOP_THRESHOLD = 0.90d;
    final double STOP = 0.0d;
    final double PICKUP_POWER = 0.8d;

    double      M_drivePowerR       = STOP,
                M_drivePowerL       = STOP,
                M_liftPowerR        = STOP,
                M_liftPowerL        = STOP,
                M_pickupPower       = STOP,
                M_basketPower       = STOP;

    final double    S_BASKET_START_POS = Servo.MIN_POSITION,
                    S_CLIMBER_KNOCKDOWN_R_START_POS = Servo.MIN_POSITION,
                    S_CLIMBER_KNOCKDOWN_L_START_POS = Servo.MAX_POSITION,
                    S_CLIMBER_DROP_DEPOSIT_START_POS = Servo.MAX_POSITION;

    final double    S_BASKET_END_POS = Servo.MAX_POSITION,
                    S_CLIMBER_KNOCKDOWN_R_END_POS = Servo.MAX_POSITION,
                    S_CLIMBER_KNOCKDOWN_L_END_POS = Servo.MIN_POSITION,
                    S_CLIMBER_DROP_DEPOSIT_END_POS = Servo.MIN_POSITION;

    final double    S_CLIMBER_DEPOSIT_SWING_FORWARD_POWER  = 0.6d,
                    S_CLIMBER_DEPOSIT_SWING_BACKWARD_POWER  = 0.4d,
                    S_CLIMBER_DEPOSIT_SWING_STOP_POWER      = 0.5d;

    double  S_basketPosition = S_BASKET_START_POS,
            S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_START_POS,
            S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_START_POS,
            S_climberDepositDropPos  = S_CLIMBER_DROP_DEPOSIT_START_POS;

    double  S_climberDepositSwingPower = S_CLIMBER_DEPOSIT_SWING_STOP_POWER;

    private double convertStick(float controllerValue) {   return Math.sin(Range.clip(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD, -Math.PI / 2, Math.PI / 2)); }

    void grabMotors() {
        M_driveFR   = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL   = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR   = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL   = hardwareMap.dcMotor.get("M_driveBL");
        M_liftR     = hardwareMap.dcMotor.get("M_liftR");
        M_liftL     = hardwareMap.dcMotor.get("M_liftL");
        M_pickup    = hardwareMap.dcMotor.get("M_pickup");
        M_basket    = hardwareMap.dcMotor.get("M_basket");
    }
    void configureMotors() {
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        M_liftR.setDirection(DcMotor.Direction.REVERSE);
        M_liftL.setDirection(DcMotor.Direction.REVERSE);
    }
    void grabServos() {
        S_basket = hardwareMap.servo.get("S_basket");
        S_climberKnockdownR = hardwareMap.servo.get("S_climberKnockdownR");
        S_climberKnockdownL = hardwareMap.servo.get("S_climberKnockdownL");
        S_climberDepositDrop = hardwareMap.servo.get("S_climberDepositDrop");
        S_climberDepositSwing = hardwareMap.servo.get("S_climberDepositSwing");
    }

    @Override
    public void init() {
        grabMotors();
        configureMotors();
        grabServos();
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        // drive base control block
        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);
        // pickup control block
        if(gamepad1.right_bumper) {
            M_pickupPower = PICKUP_POWER;
        } else if(gamepad1.left_bumper) {
            M_pickupPower = -PICKUP_POWER;
        } else {
            M_pickupPower = STOP;
        }
        // basket motor control block
        if(gamepad1.right_trigger > 0.0d) {
            M_basketPower = gamepad1.right_trigger;
        } else if(gamepad1.left_trigger > 0.0d) {
            M_basketPower = -gamepad1.left_trigger;
        } else {
            M_basketPower = STOP;
        }
        // basket servo control block
        if(gamepad1.a) {
            S_basketPosition = S_BASKET_END_POS;
        } else if(gamepad1.y) {
            S_basketPosition = S_BASKET_START_POS;
        }
        // climbers knockdown block
        if(gamepad1.dpad_up) {
            S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_END_POS;
            S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_END_POS;
        } else if(gamepad1.dpad_right) {
            if(S_climberKnockdownRPos == S_CLIMBER_KNOCKDOWN_R_START_POS) {
                S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_END_POS;
            } else {
                S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_START_POS;
            }
        } else if(gamepad1.dpad_left) {
            if(S_climberKnockdownLPos == S_CLIMBER_KNOCKDOWN_L_START_POS) {
                S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_END_POS;
            } else {
                S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_START_POS;
            }
        } else if(gamepad1.dpad_down) {
            S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_START_POS;
            S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_START_POS;
        }

        // climber swing control block
        if(gamepad2.right_trigger > 0.0d) {
            S_climberDepositSwingPower = 0.50d + (gamepad2.right_trigger / 2.0d);
        } else if(gamepad2.left_trigger > 0.0d) {
            S_climberDepositSwingPower = 0.50d - (gamepad2.left_trigger / 2.0d);
        } else {
            S_climberDepositSwingPower = 0.50d;
        }
        // climber drop control block

        if(gamepad2.a) {
            S_climberDepositDropPos = S_CLIMBER_DROP_DEPOSIT_END_POS;
        } else if(gamepad2.y) {
            S_climberDepositDropPos = S_CLIMBER_DROP_DEPOSIT_START_POS;
        }
        // lift control block
        M_liftPowerR = convertStick(-gamepad2.right_stick_y);
        M_liftPowerL = convertStick(-gamepad2.left_stick_y);

        M_driveFR.setPower(M_drivePowerR);
        M_driveFL.setPower(M_drivePowerL);
        M_driveBR.setPower(M_drivePowerR);
        M_driveBL.setPower(M_drivePowerL);
        M_liftR.setPower(M_liftPowerR);
        M_liftL.setPower(M_liftPowerL);
        M_pickup.setPower(M_pickupPower);
        M_basket.setPower(M_basketPower);

        S_basket.setPosition(S_basketPosition);
        S_climberKnockdownR.setPosition(S_climberKnockdownRPos);
        S_climberKnockdownL.setPosition(S_climberKnockdownLPos);
        S_climberDepositDrop.setPosition(S_climberDepositDropPos);

        S_climberDepositSwing.setPosition(S_climberDepositSwingPower);
    }
    @Override
    public void stop() {
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
        M_liftR.setPower(STOP);
        M_liftL.setPower(STOP);
        M_pickup.setPower(STOP);
        M_basket.setPower(STOP);
    }
}
