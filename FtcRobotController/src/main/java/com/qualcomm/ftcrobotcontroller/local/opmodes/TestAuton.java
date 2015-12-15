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
public class TestAuton extends LinearOpMode {

    // motor declarations
    DcMotor M_driveFR   = null, // front right drive motor
            M_driveFL   = null, // front left drive motor
            M_driveBR   = null, // back right drive motor
            M_driveBL   = null, // back left drive motor
            M_pickup    = null, // pickup motor
            M_lift      = null; // lift motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDepositRotate = null, // servo that rotates climbers
            S_climbersDepositDrop   = null, // servo that drops climbers
            S_basketRotate          = null, // servo that controls basket rotation
            S_basketRelease         = null, // servo that releases blocks
            S_basketTilt            = null; // servo that controls tilt
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
                    S_BASKET_TILT_START_POS             = 0.875d,
                    S_BASKET_RELEASE_START_POS          = 0.34d;


    // all of the ending/close servo positions
    final double    S_CLIMBERS_KNOCKDOWN_END_POS_R      = 0.494d,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_ROTATE_END_POS   = 0.549d,
                    S_climbers_DEPOSIT_DROP_END_POS     = Servo.MIN_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION;

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
            S_basketTiltPos             = S_BASKET_TILT_START_POS,
            S_basketReleasePos          = S_BASKET_RELEASE_START_POS;

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

        motorTargetsDrive = new int[2];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);
        Arrays.fill(motorTargetsTurn, 0);

        clock = new ElapsedTime();
    }

    private boolean waitingForClick() {
        telemetry.addData("Waiting for click", "waiting");
        if(gamepad1.a) {
            return false;
        }
        return true;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        mapStuff();
        configureStuff();
        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        double case1Time = 0;
        boolean hasBeenSet = false;
        boolean finished = false;
        drivePowers = new double[2];
        Arrays.fill(drivePowers, 0.0d);
        waitForStart();
        clock.startTime();
        boolean isSet = false;
        while(opModeIsActive()) {
            if(!isSet) {
                // 12 gives 12.5 in
                //motorTargetsDrive = setDriveTarget(24.0);
                motorTargetsTurn = setTurnTarget(180.0d);
                isSet = true;
            }
            boolean bleh = turnRight();

            M_drivePowerR = drivePowers[0];
            M_drivePowerL = drivePowers[1];
            M_driveFR.setPower(M_drivePowerR);
            M_driveFL.setPower(M_drivePowerL);
            M_driveBR.setPower(M_drivePowerR);
            M_driveBL.setPower(M_drivePowerL);

            // updates all the servo positions
            S_climbersKnockdownR.setPosition(S_climbersKnockdownPosR);
            S_climbersKnockdownL.setPosition(S_climbersKnockdownPosL);
            S_climbersDepositRotate.setPosition(S_climbersDepositRotatePos);
            S_climbersDepositDrop.setPosition(S_climbersDepositDropPos);
            //S_basketRotate.setPosition(S_basketRotateSpeed);
            S_basketRelease.setPosition(S_basketReleasePos);
            S_basketTilt.setPosition(S_basketTiltPos);

            telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
            telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
            telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
            telemetry.addData("LB POS", M_driveBL.getCurrentPosition());

            telemetry.addData("R Target", motorTargetsTurn[0]);
            telemetry.addData("L Target", motorTargetsTurn[1]);

            //telemetry.addData("Counter", counter);
            telemetry.addData("Supposed Power R", drivePowers[0]);
            telemetry.addData("Supposed Power L", drivePowers[1]);
            sleep(20);
        }
    }

    public void stopDriving() {
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

    // 12 goes 14.5

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
                /*motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
                motors[i + 2].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));*/
                drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                drivePowers[i] = 0.0d;
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
        //targets[0] = (int)((motors[0].getCurrentPosition() + motors[2].getCurrentPosition()) / 2 - degrees * DEGREES_TO_TICKS);
        //targets[1] = (int)((motors[1].getCurrentPosition() + motors[3].getCurrentPosition()) / 2 + degrees * DEGREES_TO_TICKS);
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
            //int error = (int)(motorTargetsTurn[i] - (motors[i].getCurrentPosition() - motors[i + 2].getCurrentPosition()) / 2);
            int error = motorTargetsTurn[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(actualPIDValue[i]) > 0.05) {
                /*motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
                motors[i + 2].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));*/
                drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                drivePowers[i] = 0.0d;
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
