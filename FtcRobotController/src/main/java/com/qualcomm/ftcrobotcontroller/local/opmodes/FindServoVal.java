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
    Servo[] servos = {S_climbersKnockdownR, S_climbersKnockdownL, S_climbersDeposit, S_basketRotate, S_basketRelease, S_basketTilt, S_buttonPusher};

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
        for(Servo S : servos) {
            telemetry.addData("Servo: " + S.getDeviceName() + " Position: ", S.getPosition());
        }
    }
    @Override
    public void stop() {

    }
}
