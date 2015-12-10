package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Andrew on 12/9/2015.
 */
public class FindServoVal extends OpMode {

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_buttonPusher          = null, // servo that pushes button
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_basketTilt            = null; // front left servo of the pickup
    Servo[] servos;

    @Override
    public void init() {
        // mapping servo variables to their hardware counterparts
        this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
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
            S_climbersDeposit.setPosition(S_climbersDeposit.getPosition() + 0.01d);
            S_buttonPusher.setPosition(S_buttonPusher.getPosition() + 0.01d);
            S_basketRotate.setPosition(0.5d);
            S_basketRelease.setPosition(S_basketRelease.getPosition() + 0.01d);
            S_basketTilt.setPosition(S_basketTilt.getPosition() + 0.01d);
        } else if(gamepad1.start) {
            S_climbersKnockdownR.setPosition(0.0d);
            S_climbersKnockdownL.setPosition(0.0d);
            S_climbersDeposit.setPosition(0.0d);
            S_buttonPusher.setPosition(0.0d);
            S_basketRotate.setPosition(0.5d);
            S_basketRelease.setPosition(0.0d);
            S_basketTilt.setPosition(0.0d);
        }
        telemetry.addData("Servo S_climbersKnockdownR: Position: ", S_climbersKnockdownR.getPosition());
        telemetry.addData("Servo S_climbersKnockdownL: Position: ", S_climbersKnockdownL.getPosition());
        telemetry.addData("Servo S_climbersDeposit: Position: ", S_climbersDeposit.getPosition());
        telemetry.addData("Servo S_buttonPusher: Position: ", S_buttonPusher.getPosition());
        telemetry.addData("Servo S_basketRotate: Position: ", S_basketRotate.getPosition());
        telemetry.addData("Servo S_basketRelease: Position: ", S_basketRelease.getPosition());
        telemetry.addData("Servo S_basketTilt: Position: ", S_basketTilt.getPosition());

    }
    @Override
    public void stop() {

    }
}
