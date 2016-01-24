package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.ftcrobotcontroller.local.lib.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Anjali on 1/15/2016.
 */
public class DebrisAuton extends LinearOpMode {

    DcMotor M_driveBR,
            M_driveBL,
            M_driveFR,
            M_driveFL;
            /*M_liftR     = null,
            M_liftL     = null,
            M_pickup    = null,
            M_basket    = null;*/
    /*Servo   buttonPusher        = null,
            climberDrop         = null,
            climberKnockdownR   = null,
            climberKnockdownL   = null,
            colorSensorGround   = null;*/
    ColorSensor whiteTapeSensor,
                beaconSensor;

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
    double[]        drivePowers;

    void grabMotors() {
        M_driveFR   = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL   = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR   = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL   = hardwareMap.dcMotor.get("M_driveBL");
        //M_liftR     = hardwareMap.dcMotor.get("M_liftR");
        //M_liftL     = hardwareMap.dcMotor.get("M_liftL");
        //M_pickup    = hardwareMap.dcMotor.get("M_pickup");
        //M_basket    = hardwareMap.dcMotor.get("M_basket");
    }
    void configureMotors() {
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        //M_liftR.setDirection(DcMotor.Direction.REVERSE);
    }
    void grabServos() {

    }
    void grabSensors() {

    }

    void setPID() {
        drivePID = new PIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, DRIVE_POWER_MIN, PIDController.TypePID.DRIVE, drivePowers, M_driveFR, M_driveBR, M_driveFL, M_driveBL);
        turnPID = new PIDController(DRIVE_WHEEL_DIAMETER, TURN_DIAMETER, DRIVE_GEAR_RATIO, TURN_THRESHOLD, TURN_SLOW_DOWN_START, TURN_FINE_TUNE_START, TURN_POWER_MIN, PIDController.TypePID.TURN, drivePowers, M_driveFR, M_driveBR, M_driveFL, M_driveBL);
    }
    private boolean waitingForClick() {
        telemetry.addData("Waiting for click", "waiting");
        if(gamepad1.a) {
            return false;
        }
        return true;
    }

    PIDController drivePID;
    PIDController turnPID;

    @Override
    public void runOpMode() throws InterruptedException {
        drivePowers = new double[2];
        grabMotors();
        configureMotors();
        grabServos();
        grabSensors();
        setPID();

        int counter = 0;
        int tempPosition[] = new int[2];
        boolean hasBeenSet = false;

        waitForStart();
        while(opModeIsActive()) {
            switch (counter) {
                case 0:
                    if(!hasBeenSet) {
                        drivePID.setTargets(51.0d);
                        hasBeenSet = true;
                    }
                    drivePowers = drivePID.run();
                    if(drivePID.hasReachedDestination()) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            telemetry.addData("Drive R Pos: ", drivePID.getCurrentPosition()[0]);
                            telemetry.addData("Drive L Pos: ", drivePID.getCurrentPosition()[1]);
                            telemetry.addData("target R: ", drivePID.getTargets()[0]);
                            telemetry.addData("target L: ", drivePID.getTargets()[1]);
                            sleep(100);
                        }
                    }
                    break;
                case 1:
                    if(!hasBeenSet) {
                        turnPID.setTargets(90.0d);
                        hasBeenSet = true;
                    }
                    drivePowers = turnPID.run();
                    if(turnPID.hasReachedDestination()) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                /*case 2:
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
                        tempPosition = drivePID.getCurrentPosition();
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    }
                    break;
                case 5:
                    if(whiteTapeSensor.red() > 0.8 && whiteTapeSensor.blue() > 0.8d && whiteTapeSensor.green() > 0.8d) {
                        stopRobot();
                        counter++;
                        while(waitingForClick()) {
                            sleep(100);
                        }
                    } else {
                        M_driveFR.setPower(DRIVE_POWER_MIN);
                        M_driveBR.setPower(DRIVE_POWER_MIN);
                        M_driveFL.setPower(DRIVE_POWER_MIN);
                        M_driveBL.setPower(DRIVE_POWER_MIN);
                    }
                    break;
                case 6:
                    // do some things to make sure robot gets colors sensor
                    counter++;
                    break;
                case 7:
                    // extend color sensor, make center,
                    counter++;
                    break;
                case 8:
                    break;
                    */
                default:
                    break;
            }
            //M_pickup.setPower(M_pickupPower);
            M_driveFR.setPower(drivePowers[0]);
            M_driveBR.setPower(drivePowers[0]);
            M_driveFL.setPower(drivePowers[1]);
            M_driveBL.setPower(drivePowers[1]);
            telemetry.addData("Drive R Pos: ", drivePID.getCurrentPosition()[0]);
            telemetry.addData("Drive L Pos: ", drivePID.getCurrentPosition()[1]);
            sleep(100);
        }
    }
    void stopRobot() {
        M_driveFR.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBL.setPower(STOP);

    }
}
