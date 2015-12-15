package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/**
 * Created by Anjali on 12/12/2015.
 */

// 25 second auton
public class RedAutonNearDelay extends LinearOpMode {
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
            S_climbersDepositRotate = null, // servo that rotates climbers
            S_climbersDepositDrop   = null, // servo that drops climbers
            S_basketRotate          = null, // servo that controls basket rotation
            S_basketRelease         = null, // servo that releases blocks
            S_basketTilt            = null, // servo that controls tilt
            S_buttonPusher          = null; // servo that pushes button

    // sensor declarations
    ColorSensor colorSensor = null; // color sensor

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
            S_CLIMBERS_DEPOSIT_ROTATE_START_POS = Servo.MIN_POSITION,
            S_climbers_DEPOSIT_DROP_START_POS   = Servo.MAX_POSITION,
            S_BUTTON_PUSHER_START_POS           = Servo.MIN_POSITION;

    // all of the ending/close servo positions
    final double    S_CLIMBERS_DEPOSIT_ROTATE_END_POS   = 0.549d,
            S_climbers_DEPOSIT_DROP_END_POS     = Servo.MIN_POSITION,
            S_BUTTON_PUSHER_END_POS             = 0.496d;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;
    double[] drivePowers;

    // servo positions
    double  S_climbersKnockdownPosR     = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL     = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositRotatePos  = S_CLIMBERS_DEPOSIT_ROTATE_START_POS,
            S_climbersDepositDropPos    = S_climbers_DEPOSIT_DROP_START_POS,
            S_buttonPusherPos           = S_BUTTON_PUSHER_START_POS;

    // function necessity delcarations
    int[] motorTargetsDrive;
    int[] motorTargetsTurn;

    ElapsedTime clock;

    private void mapStuff() {
        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");

        // mapping servo variables to their hardware counterparts
        S_climbersKnockdownR    = hardwareMap.servo.get("S_climbersKnockdownR");
        S_climbersKnockdownL    = hardwareMap.servo.get("S_climbersKnockdownL");
        S_climbersDepositRotate = hardwareMap.servo.get("S_climbersDepositRotate");
        S_climbersDepositDrop   = hardwareMap.servo.get("S_climbersDepositDrop");
        S_basketRotate          = hardwareMap.servo.get("S_basketRotate");
        S_basketRelease         = hardwareMap.servo.get("S_basketRelease");
        S_basketTilt            = hardwareMap.servo.get("S_basketTilt");
        S_buttonPusher          = hardwareMap.servo.get("S_buttonPusher");

        colorSensor            = hardwareMap.colorSensor.get("S_colorSensor");
    }

    private void configureStuff() {
        // fixing motor directions
        this.M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        this.M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        this.M_pickup.setDirection(DcMotor.Direction.REVERSE);
        this.M_lift.setDirection(DcMotor.Direction.REVERSE);

        // resets all the encoder values
        this.M_driveFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_lift.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        this.M_driveFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_lift.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorTargetsDrive = new int[2];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);
        Arrays.fill(motorTargetsTurn, 0);

        clock = new ElapsedTime();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mapStuff();
        configureStuff();
        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        boolean hasBeenSet = false;
        boolean finished = false;
        drivePowers = new double[2];
        Arrays.fill(drivePowers, 0.0d);
        waitForStart();
        clock.startTime();
        int tempMotorPosR = 0;
        double increment = 0.05d;
        while(opModeIsActive()) {

            switch (counter) {
                case 0:
                    if(!hasBeenSet) {
                        clock.reset();
                        hasBeenSet = true;
                    }
                    if(isPastTime(5.0d)) {
                        counter++;
                        hasBeenSet = false;
                        sleep(200);
                    }
                    break;
                case 1:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(19.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if(finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;

                case 2:
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-42.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if(finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 3:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(62.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if(finished || isPastTime(3.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 4:
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(42.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if(finished  || isPastTime(1.0d)) {
                        tempMotorPosR = M_driveFR.getCurrentPosition();
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 5:
                    if(!isRed()) {
                        M_drivePowerR = 0.2d;
                        M_drivePowerL = 0.2d;
                    } else {
                        stopDriving();
                        counter++;
                        sleep(200);
                    }
                    break;
                case 6:
                    if(Math.abs(S_buttonPusherPos - S_BUTTON_PUSHER_END_POS) < increment && Math.abs(S_climbersDepositRotatePos - S_CLIMBERS_DEPOSIT_ROTATE_END_POS) < increment) {
                        counter++;
                        sleep(200);
                    } else {
                        if (Math.abs(S_buttonPusherPos - S_BUTTON_PUSHER_END_POS) > increment + 0.01d) {
                            S_buttonPusherPos += increment;
                        }
                        if (Math.abs(S_climbersDepositRotatePos - S_CLIMBERS_DEPOSIT_ROTATE_END_POS) > increment + 0.01d) {
                            S_climbersDepositRotatePos += increment;
                        }
                    }
                    break;
                case 7:
                    if(Math.abs(S_climbersDepositDropPos - S_climbers_DEPOSIT_DROP_END_POS) < increment) {
                        counter++;
                        sleep(200);
                    } else {
                        S_climbersDepositDropPos -= increment;
                    }
                    clock.reset();
                    break;
                case 8:
                    if((Math.abs(S_buttonPusherPos - S_BUTTON_PUSHER_START_POS) <= increment && Math.abs(S_climbersDepositRotatePos - S_CLIMBERS_DEPOSIT_ROTATE_START_POS) <= increment && Math.abs(S_climbersDepositDropPos - S_climbers_DEPOSIT_DROP_START_POS) <= increment || isPastTime(0.5d))) {
                        counter++;
                        sleep(200);
                    } else {
                        if(Math.abs(S_climbersDepositDropPos - S_climbers_DEPOSIT_DROP_START_POS) > increment + 0.01d) {
                            S_climbersDepositDropPos += increment;
                        }
                        if(Math.abs(S_climbersDepositRotatePos - S_CLIMBERS_DEPOSIT_ROTATE_START_POS) > increment + 0.01d) {
                            S_climbersDepositRotatePos -= increment;
                        }
                        if(Math.abs(S_buttonPusherPos - S_BUTTON_PUSHER_START_POS) > increment + 0.01d) {
                            S_buttonPusherPos -= increment;
                        }
                        telemetry.addData("Drop POS", S_climbersDepositDropPos);
                        telemetry.addData("Rotate POS", S_climbersDepositRotatePos);
                        telemetry.addData("Push POS", S_buttonPusherPos);
                    }
                    break;
                case 9:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(tempMotorPosR - 200 - M_driveFL.getCurrentPosition());
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if(finished || isPastTime(1.5d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 10:
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-47.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if(finished  || isPastTime(1.0d)) {
                        tempMotorPosR = M_driveFR.getCurrentPosition();
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 11:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-20.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if(finished || isPastTime(1.5d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 12:
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(90.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if(finished  || isPastTime(2.0d)) {
                        tempMotorPosR = M_driveFR.getCurrentPosition();
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        sleep(200);
                    }
                    break;
                case 13:
                    if(!hasBeenSet) {
                        clock.reset();
                        hasBeenSet = true;
                    }
                    M_drivePowerR = -MAX_POWER;
                    M_drivePowerL = -MAX_POWER;
                    if(isPastTime(2.0d)) {
                        M_drivePowerR = 0;
                        M_drivePowerL = 0;
                        counter++;
                    }
                    break;
                default:
                    M_drivePowerR = STOP;
                    M_drivePowerL = STOP;
                    this.M_pickup.setPower(STOP);
                    this.M_lift.setPower(STOP);
                    telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                    telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                    telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                    telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                    telemetry.addData("Target R", motorTargetsDrive[0]);
                    telemetry.addData("Target L", motorTargetsDrive[1]);
                    break;
            }
            M_driveFR.setPower(M_drivePowerR);
            M_driveFL.setPower(M_drivePowerL);
            M_driveBR.setPower(M_drivePowerR);
            M_driveBL.setPower(M_drivePowerL);

            S_climbersDepositRotate.setPosition(S_climbersDepositRotatePos);
            S_climbersDepositDrop.setPosition(S_climbersDepositDropPos);
            S_buttonPusher.setPosition(S_buttonPusherPos);
            S_climbersKnockdownL.setPosition(S_climbersKnockdownPosL);

            telemetry.addData("Counter", counter);
            telemetry.addData("Supposed Power R", M_drivePowerR);
            telemetry.addData("Supposed Power L", M_drivePowerL);
            telemetry.addData("time", clock.time());
            sleep(20);
        }
    }

    private boolean isRed() {   return colorSensor.red() > colorSensor.blue();  }

    public boolean isPastTime(double maxTime) {
        if(clock.time() > maxTime) {
            return true;
        } else {
            return false;
        }
    }

    public void stopDriving() {
        M_drivePowerR = STOP;
        M_drivePowerL = STOP;
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
    }

    public int[] setDriveTarget(double inches) {
        final double INCHES_TO_TICKS = 1100.0d / 12.0d;
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        int[] targets = new int[2];
        targets[0] = (int)(motors[0].getCurrentPosition() + inches * INCHES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + inches * INCHES_TO_TICKS);
        return targets;
    }

    // 12 goes 12.5

    public boolean driveForward() {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.002d;
        double kI;
        double[] actualPIDValue = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            int error = motorTargetsDrive[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(actualPIDValue[i]) > thresholdPower) {
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
            } else {
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
            }
        }
        if(Math.abs((M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2.0d - motorTargetsDrive[0]) > 30) {
            return false;
        } else if(Math.abs((M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2.0d - motorTargetsDrive[1]) > 30) {
            return false;
        }
        return true;
    }

    public int[] setTurnTarget(double degrees) {
        final double DEGREES_TO_TICKS = 1160.0d / 90.0d;
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        int[] targets = new int[2];
        targets[0] = (int)(motors[0].getCurrentPosition() - degrees * DEGREES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + degrees * DEGREES_TO_TICKS);
        return targets;
    }

    public boolean turnRight() {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.002d;
        double kI;
        double actualPIDValue[] = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            int error = motorTargetsTurn[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(actualPIDValue[i]) > 0.05) {
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
            } else {
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
            }
        }
        if(Math.abs((M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2.0d - motorTargetsTurn[0]) > 30) {
            return false;
        } else if(Math.abs((M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2.0d - motorTargetsTurn[1]) > 30) {
            return false;
        }
        return true;
    }
}
