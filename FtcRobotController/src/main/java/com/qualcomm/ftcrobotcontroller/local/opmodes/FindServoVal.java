package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// button start Servo.Min
// button end 0.635d


/**
 * Created by Andrew on 12/9/2015.
 */
public class FindServoVal extends OpMode {

    // servo declarations
    Servo
            S_climberDropSwing,
            S_climberDropDeposit,
            S_climberKnockdownR,
            S_climberKnockdownL,
            S_basket;

    private double setServoSpeed(Servo servo, double targetPos, double power) {
        if (servo.getPosition() > targetPos) {
            power -= 0.01d;
        } else {
            power += 0.01d;
        }
        return Range.clip(power, 0.0d, 1.0d);
    }

    @Override
    public void init() {
        // mapping servo variables to their hardware counterparts
        S_climberDropSwing = hardwareMap.servo.get("S_climberDepositSwing");
        S_climberDropDeposit = hardwareMap.servo.get("S_climberDepositDrop");
        S_climberKnockdownR = hardwareMap.servo.get("S_climberKnockdownR");
        S_climberKnockdownL = hardwareMap.servo.get("S_climberKnockdownL");
        S_basket            = hardwareMap.servo.get("S_basket");
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        if(gamepad1.a && S_climberKnockdownR.getPosition() < 0.99d) {
            S_climberDropDeposit.setPosition(S_climberDropSwing.getPosition() + 0.01d);
            S_climberDropDeposit.setPosition(S_climberDropDeposit.getPosition() + 0.01d);
            S_climberKnockdownR.setPosition(S_climberKnockdownR.getPosition() + 0.01d);
            S_climberKnockdownL.setPosition(S_climberKnockdownL.getPosition() + 0.01d);
            S_basket.setPosition(S_basket.getPosition() + 0.01d);
            //S_basketRotate.setPosition(0.5d);

        } else if(gamepad1.start) {
            S_climberDropSwing.setPosition(0.0d);
            S_climberDropDeposit.setPosition(0.0d);
            S_climberKnockdownR.setPosition(0.0d);
            S_climberKnockdownL.setPosition(0.0d);
            S_basket.setPosition(0.0d);
        }
        if(gamepad1.b) {
            S_climberDropSwing.setPosition(0.60d);
        } else if(gamepad1.x) {
            S_climberDropSwing.setPosition(0.40d);
        } else {
            S_climberDropSwing.setPosition(0.5d);
        }
        telemetry.addData("Servo S_climbersKnockdownR: Position: ", S_climberKnockdownR.getPosition());
        telemetry.addData("Servo S_climbersKnockdownL: Position: ", S_climberKnockdownL.getPosition());
        telemetry.addData("Servo S_climberDeposit: Position: ", S_climberDropDeposit.getPosition());
        telemetry.addData("Servo S_climberSwing: Position: ", S_climberDropSwing.getPosition());
        telemetry.addData("Servo S_basket: Position: ", S_basket.getPosition());

    }
    @Override
    public void stop() {

    }
}
