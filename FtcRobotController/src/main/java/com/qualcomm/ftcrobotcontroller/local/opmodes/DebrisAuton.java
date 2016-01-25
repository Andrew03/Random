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

    // motor declarations
    DcMotor M_driveBR   = null,
            M_driveBL   = null,
            M_driveFR   = null,
            M_driveFL   = null,
            M_liftR     = null,
            M_liftL     = null,
            M_pickup    = null,
            M_basket    = null;

    // servo declarations
    Servo   //S_buttonPusher        = null,
            S_climberDrop         = null,
            S_climberKnockdownR   = null,
            S_climberKnockdownL   = null;
            //S_colorSensorGround   = null;
    ColorSensor whiteTapeSensor,
                beaconSensor;

    // constant powers
    final double    STOP            = 0.0d,
                    DRIVE_POWER     = 0.8d,
                    TURN_POWER      = 0.8d,
                    PICKUP_POWER    = 0.8d;

    // starting servo positions
    final double    S_CLIMBER_DROP_START_POS        = 0.0d,
                    S_CLIMBER_KNOCKDOWN_R_START_POS = 0.0d,
                    S_CLIMBER_KNOCKDOWN_L_START_POS = 0.0d;

    // ending servo positions
    final double    S_CLIMBER_DROP_END_POS        = 0.0d,
                    S_CLIMBER_KNOCKDOWN_R_END_POS = 0.0d,
                    S_CLIMBER_KNOCKDOWN_L_END_POS = 0.0d;

    // drive and turn PID constants
    final int       DRIVE_WHEEL_DIAMETER    = 4,
                    DRIVE_GEAR_RATIO        = 1,
                    DRIVE_THRESHOLD         = 20,
                    TURN_THRESHOLD          = 80;
    final double    DRIVE_SLOW_DOWN_START   = 1.5d,
                    DRIVE_FINE_TUNE_START   = 0.5d,
                    DRIVE_POWER_MIN         = 0.25d,
                    TURN_SLOW_DOWN_START    = 1.5d,
                    TURN_FINE_TUNE_START    = 0.5d,
                    TURN_POWER_MIN          = 0.25d,
                    TURN_DIAMETER           = Math.sqrt(Math.pow(9.0d, 2.0d) + Math.pow(16.5d, 2.0d));

    // drive powers
    double          M_drivePowerR           = STOP,
                    M_drivePowerL           = STOP,
                    M_liftPowerR            = STOP,
                    M_liftPowerL            = STOP,
                    M_pickupPower           = STOP,
                    M_basketPower           = STOP;
    double[]        drivePowers;

    // servo positions
    double          S_climberDropPos        = S_CLIMBER_DROP_START_POS,
                    S_climberKnockdownRPos  = S_CLIMBER_KNOCKDOWN_R_START_POS,
                    S_climberKnockdownLPos  = S_CLIMBER_KNOCKDOWN_L_START_POS;

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
        S_climberDrop         = hardwareMap.servo.get("S_climberDrop");
        S_climberKnockdownR   = hardwareMap.servo.get("S_climberKnockdownR");
        S_climberKnockdownL   = hardwareMap.servo.get("S_climberKnockdownL");
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
                        // go 51
                        drivePID.setTargets(12.0d);
                        hasBeenSet = true;
                    }
                    drivePowers = drivePID.run();
                    if(drivePID.hasReachedDestination()) {
                        hasBeenSet = false;
                        counter++;
                        while(waitingForClick()) {
                            stopRobot();
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
            M_pickup.setPower(-PICKUP_POWER);
            telemetry.addData("Drive R Pos: ", drivePID.getCurrentPosition()[0]);
            telemetry.addData("Drive L Pos: ", drivePID.getCurrentPosition()[1]);
            telemetry.addData("Drive R Target: ", drivePID.getTargets()[0]);
            telemetry.addData("Drive L Target: ", drivePID.getTargets()[1]);
            telemetry.addData("Drive R Power: ", drivePowers[0]);
            telemetry.addData("Drive L Power: ", drivePowers[1]);
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
