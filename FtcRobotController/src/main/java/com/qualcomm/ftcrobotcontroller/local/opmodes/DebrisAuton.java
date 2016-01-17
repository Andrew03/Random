package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.ftcrobotcontroller.local.lib.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Anjali on 1/15/2016.
 */
public class DebrisAuton extends LinearOpMode {

    DcMotor M_driveBR,
            M_driveBL,
            M_driveFR,
            M_driveFL,
            M_liftR,
            M_liftL,
            M_pickup,
            M_basket;
    Servo buttonPusher,
            climberDropSwing,
            climberDropDeposit,
            climberKnockdown;

    final double    STOP = 0.0d;
    final double    DRIVE_POWER     = 0.8d,
                    TURN_POWER      = 0.8d,
                    PICKUP_POWER    = 0.8d;
    final int       DRIVE_WHEEL_DIAMETER    = 4,
                    DRIVE_GEAR_RATIO        = 1,
                    DRIVE_THRESHOLD         = 20,
                    TURN_THRESHOLD          = 20;
    final double    DRIVE_SLOW_DOWN_START   = 4.0d,
                    DRIVE_FINE_TUNE_START   = 1.0d,
                    DRIVE_POWER_MIN         = 0.2d,
                    TURN_SLOW_DOWN_START    = 3.0d,
                    TURN_FINE_TUNE_START    = 1.0d,
                    TURN_POWER_MIN          = 0.20d,
                    TURN_DIAMETER           = 3.0d;
    double          M_drivePowerR           = STOP,
                    M_drivePowerL           = STOP,
                    M_liftPowerR            = STOP,
                    M_liftPowerL            = STOP,
                    M_pickupPower           = STOP,
                    M_basketPower           = STOP;

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
    }
    void grabServos() {

    }
    void grabSensors() {

    }

    private boolean waitingForClick() {
        telemetry.addData("Waiting for click", "waiting");
        if(gamepad1.a) {
            return false;
        }
        return true;
    }

    PIDController drivePID = new PIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, DRIVE_POWER_MIN, PIDController.TypePID.DRIVE, M_driveFR, M_driveBR, M_driveFL, M_driveBL);
    PIDController turnPID  = new PIDController(DRIVE_WHEEL_DIAMETER, TURN_DIAMETER, DRIVE_GEAR_RATIO, TURN_THRESHOLD, TURN_SLOW_DOWN_START, TURN_FINE_TUNE_START, TURN_POWER_MIN, PIDController.TypePID.TURN, M_driveFR, M_driveBR, M_driveFL, M_driveBL);
    @Override
    public void runOpMode() throws InterruptedException {
        grabMotors();
        configureMotors();
        grabServos();
        grabSensors();

        int counter = 0;
        int tempPosition[] = new int[2];
        boolean hasBeenSet = false;
        boolean finished = false;

        while(opModeIsActive()) {
            switch (counter) {
                case 0:
                    if(!hasBeenSet) {
                        drivePID.setTargets(51.0d);
                        hasBeenSet = true;
                    }
                    finished = drivePID.run();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 1:
                    if(!hasBeenSet) {
                        turnPID.setTargets(90.0d);
                        hasBeenSet = true;
                    }
                    finished = turnPID.run();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 2:
                    if(!hasBeenSet) {
                        drivePID.setTargets(48.0d);
                        hasBeenSet = true;
                    }
                    finished = drivePID.run();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 3:
                    if(!hasBeenSet) {
                        drivePID.setTargets(90.0d);
                        hasBeenSet = true;
                    }
                    finished = drivePID.run();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 4:
                    if(!hasBeenSet) {
                        drivePID.setTargets(12.0d);
                        hasBeenSet = true;
                    }
                    finished = drivePID.run();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        tempPosition[0] = (int)((M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2.0d);
                        tempPosition[1] = (int)((M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2.0d);
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 5:

            }
            sleep(100);
        }
    }
}
