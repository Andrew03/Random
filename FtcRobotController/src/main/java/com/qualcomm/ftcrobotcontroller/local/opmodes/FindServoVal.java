package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 12/9/2015.
 */
public class FindServoVal extends OpMode {

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDepositRotate = null, // servo that rotates climbers
            S_climbersDepositDrop   = null, // servo that drops climbers
            S_buttonPusher          = null, // servo that pushes button
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_basketTilt            = null; // front left servo of the pickup
    Servo[] servos;

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
        this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        S_climbersDepositRotate = hardwareMap.servo.get("S_climbersDepositRotate");
        S_climbersDepositDrop   = hardwareMap.servo.get("S_climbersDepositDrop");
        this.S_buttonPusher         = this.hardwareMap.servo.get("S_buttonPusher");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        if(gamepad1.a && S_climbersKnockdownR.getPosition() < 0.99d) {
            S_climbersKnockdownR.setPosition(S_climbersKnockdownR.getPosition() + 0.01d);
            S_climbersKnockdownL.setPosition(S_climbersKnockdownL.getPosition() + 0.01d);
            S_climbersDepositRotate.setPosition(S_climbersDepositRotate.getPosition() + 0.01d);
            S_climbersDepositDrop.setPosition(S_climbersDepositDrop.getPosition() + 0.01d);
            S_buttonPusher.setPosition(S_buttonPusher.getPosition() + 0.01d);
            //S_basketRotate.setPosition(0.5d);

            S_basketRelease.setPosition(S_basketRelease.getPosition() + 0.01d);
            S_basketTilt.setPosition(S_basketTilt.getPosition() + 0.01d);
        } else if(gamepad1.start) {
            S_climbersKnockdownR.setPosition(0.0d);
            S_climbersKnockdownL.setPosition(0.0d);
            S_climbersDepositRotate.setPosition(0.0d);
            S_climbersDepositDrop.setPosition(0.0d);
            S_buttonPusher.setPosition(0.0d);
            S_basketRotate.setPosition(0.5d);
            S_basketRelease.setPosition(0.0d);
            S_basketTilt.setPosition(0.0d);
        }
        if(gamepad1.b) {
            S_basketRotate.setPosition(0.56d);
        } else if(gamepad1.x) {
            S_basketRotate.setPosition(0.47d);
        } else {
            S_basketRotate.setPosition(0.5d);
        }
        telemetry.addData("Servo S_climbersKnockdownR: Position: ", S_climbersKnockdownR.getPosition());
        telemetry.addData("Servo S_climbersKnockdownL: Position: ", S_climbersKnockdownL.getPosition());
        telemetry.addData("Servo S_climbersDepositRotate: Position: ", S_climbersDepositRotate.getPosition());
        telemetry.addData("Servo S_climbersDepositDrop: Position: ", S_climbersDepositDrop.getPosition());
        telemetry.addData("Servo S_buttonPusher: Position: ", S_buttonPusher.getPosition());
        telemetry.addData("Servo S_basketRotate: Position: ", S_basketRotate.getPosition());
        telemetry.addData("Servo S_basketRelease: Position: ", S_basketRelease.getPosition());
        telemetry.addData("Servo S_basketTilt: Position: ", S_basketTilt.getPosition());

    }
    @Override
    public void stop() {

    }
}
