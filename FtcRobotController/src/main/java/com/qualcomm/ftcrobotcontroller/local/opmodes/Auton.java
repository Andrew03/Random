package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.ftcrobotcontroller.local.lib.Drive;
import com.qualcomm.ftcrobotcontroller.local.lib.PID;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/**
 * Created by Andrew on 10/31/2015.
 */
public class Auton extends LinearOpMode {

    // motor declarations
    DcMotor M_driveFR   = null, // front right drive motor
            M_driveFL   = null, // front left drive motor
            M_driveBR   = null, // back right drive motor
            M_driveBL   = null, // back left drive motor
            M_pickup    = null, // pickup motor
            M_lift      = null, // lift motor
            M_hangR     = null, // right hang motor
            M_hangL     = null; // left hang motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_liftR                 = null, // right servo that supports lift
            S_liftL                 = null, // left servo that supports lift
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_pickupFL              = null, // front left servo of the pickup
            S_pickupSR              = null, // servo on right side of the pickup
            S_pickupSL              = null, // servo on left side of the pickup
            S_hitchR                = null, // right hitch servo
            S_hitchL                = null; // left hitch servo
    // all of the important constants
    final double    STOP                   = 0.0d,
                    MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER    = 0.8d,
                    LIFT_POWER      = 1.0d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_LIFT_START_POS_R                  = Servo.MIN_POSITION,
                    S_LIFT_START_POS_L                  = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_START_POS           = 0.67d,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
                    S_PICKUP_START_POS_FL               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SR               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SL               = Servo.MIN_POSITION,
                    S_HITCH_START_POS_R                 = Servo.MIN_POSITION,
                    S_HITCH_START_POS_L                 = Servo.MIN_POSITION;

    // all of the ending/close servo positions
    final double    S_CLIMBERS__KNOCKDOWN_END_POS_R     = Servo.MAX_POSITION,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MAX_POSITION,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_LIFT_END_POS_R                    = Servo.MAX_POSITION,
                    S_LIFT_END_POS_L                    = Servo.MAX_POSITION,
                    S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_FR                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_FL                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SR                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SL                 = Servo.MAX_POSITION,
                    S_HITCH_END_POS_R                   = Servo.MAX_POSITION,
                    S_HITCH_END_POS_L                   = Servo.MAX_POSITION;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_liftPosR               = S_LIFT_START_POS_R,
            S_liftPosL               = S_LIFT_START_POS_L,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_pickupPosFL            = S_PICKUP_START_POS_FL,
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

    // function necessity delcarations
    int[] motorTargetsDrive;
    int[] motorTargetsTurn;


    @Override
    public void runOpMode() throws InterruptedException {
        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");
        //this.M_hangR    = this.hardwareMap.dcMotor.get("M_hangR");
        //this.M_hangL    = this.hardwareMap.dcMotor.get("M_hangL");

        // mapping servo variables to their hardware counterparts
        //this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        //this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        //this.S_liftR                = this.hardwareMap.servo.get("S_liftR");
        //this.S_liftL                = this.hardwareMap.servo.get("S_liftL");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        //this.S_pickupFL             = this.hardwareMap.servo.get("S_pickupFL");
        //this.S_pickupSR             = this.hardwareMap.servo.get("S_pickupSR");
        //this.S_pickupSL             = this.hardwareMap.servo.get("S_pickupSL");
        //this.S_hitchR               = this.hardwareMap.servo.get("S_hitchR");
        //this.S_hitchL               = this.hardwareMap.servo.get("S_hitchL");

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

        motorTargetsDrive = new int[4];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);

        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        boolean hasBeenSet = false;
        waitForStart();
        while(opModeIsActive()) {

            switch (counter) {
                case 0:
                    //setDriveTarget(motorTargetsDrive, 10.0d);
                    //counter += driveForward();
                    //motorTargetsDrive = setDriveTarget(10.0d);
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(10.0d);
                        hasBeenSet = true;
                    }
                    counter += driveForward();
                    for(int i = 0; i < 4; i++) {
                        telemetry.addData("Targets!", motorTargetsDrive[i]);
                    }
                    telemetry.addData("Counter val", counter);
                    break;
                case 1:
                    //setTurnTarget(motorTargetsTurn, 90.0d);
                    //counter += turnRight();
                    telemetry.addData("Counter changed!", counter);
                    this.M_driveBR.setPower(STOP);
                    this.M_driveBL.setPower(STOP);
                    this.M_driveFR.setPower(STOP);
                    this.M_driveFL.setPower(STOP);
                    this.M_pickup.setPower(STOP);
                    this.M_lift.setPower(STOP);
                    break;
                default:
                    this.M_driveBR.setPower(STOP);
                    this.M_driveBL.setPower(STOP);
                    this.M_driveFR.setPower(STOP);
                    this.M_driveFL.setPower(STOP);
                    this.M_pickup.setPower(STOP);
                    this.M_lift.setPower(STOP);
                    break;
            }
            waitOneFullHardwareCycle();
        }
    }
    public void setDriveTarget(int[] targets, double inches) {
        final double INCHES_TO_TICKS = 100.0d;
        for(int x : targets) {
            x += inches * INCHES_TO_TICKS;
        }
    }
    public int[] setDriveTarget(double inches) {
        final double INCHES_TO_TICKS = 10.0d;
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        int[] targets = new int[4];
        for(int i = 0; i < 4; i++) {
            targets[i] = (int)(motors[i].getCurrentPosition() + inches * INCHES_TO_TICKS);
        }
        return targets;
    }

    public int driveForward() {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double[] PIDValue = new double[4];
        double[] accumError = new double[4];
        double kP = 1.0d;
        double kI;
        double[] actualPIDValue = new double[4];
        double thresholdPower = 0.1d;

        for (int i = 0; i < motors.length; i++) {
            int error = motorTargetsDrive[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            //accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(Range.clip(actualPIDValue[i], -1.0d, 1.0d)) > thresholdPower) {
                motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
            } else {
                motors[i].setPower(0.0d);
            }
        }
        for(int i = 0; i < motors.length; i++) {
            if(Math.abs(Range.clip(actualPIDValue[i], -1.0d, 1.0d)) > thresholdPower) {
                return 0;
            }
        }
        return 1;
    }

    public void setTurnTarget(int[] targets, double degrees) {
        final double DEGREES_TO_TICKS = 1.0d;
        targets[0] += degrees * DEGREES_TO_TICKS;
        targets[1] -= degrees * DEGREES_TO_TICKS;
    }

    public int turnRight() {
        final double DEGREES_TO_TICKS = 10.0d;
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.003d;
        double kI;
        double actualPIDValue;
        double thresholdPower = 0.1d;

        for (int i = 0; i < motors.length / 2; i++) {
            int error = (int)((2 * motorTargetsTurn[i] - motors[i].getCurrentPosition() - motors[i + 2].getCurrentPosition()) / 2);
            PIDValue[i] = kP * error;
            //accumError[i] += error;
            actualPIDValue = kP * error;
            if (Math.abs(Range.clip(actualPIDValue, -1.0d, 1.0d)) > thresholdPower) {
                motors[i].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
                motors[i + 2].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
            } else {
                motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);
            }
        }
        for(DcMotor x : motors) {
            if(x.getPower() > 0.0d) {
                return 0;
            }
        }
        return 1;
    }

    /*private class DriveThread extends PID {
        public DriveThread(int gearRatio, int objectCircumference) {
            GEAR_RATIO = gearRatio;
            OBJECT_CIRCUMFERENCE = objectCircumference;
            kP = 0.0f;
            kI = 0.0f;
            kD = 0.0f;
            maxPower = 1.0f;
            minPower = -1.0f;
            minPIDPower = 0.2f;
            acceptableError = 50;
        }
        public void setTarget(float target) {
            this.target = target;
        }
    }

    private class DriveThreadRight implements Runnable {

         float  kP = 0.0f,
                kI = 0.0f,
                kD = 0.0f;
         float  maxPower = 1.0f,
                minPower = -1.0f,
                minPIDPower = 0.2f;
         int    acceptableError = 50;  // in encoder ticks
         float  target;           // in inches
         boolean isMoving = true,
                isFineTune = false;
         double  timer = 0.0f,   // in milliseconds
                fineTuneTimer = 0.0f,
                currDt = 0.0f;
         float   currError = 0.0f,   // in encoder ticks
                prevError = 0.0f,   // in encoder ticks
                errorRate = 0.0f,   // in encoder ticks
                accumError = 0.0f;  // in encoder ticks
         float   PIDValue = 0.0f,
                power = 0.0f;

         final int PULSE_PER_REV = 1120; // encoder ticks per revolution
         final double GEAR_RATIO = RIGHT_GEAR_RATIO,
                        OBJECT_CIRCUMFERENCE = RIGHT_WHEEL_CIRCUMFERENCE; // in inches
         ElapsedTime clock;

        @Override
        public void run() {
            clock.reset();
            clock.startTime();
            try {
                while(isMoving) {
                    currDt = clock.time() / 1000.0d;
                    clock.reset();
                    prevError = currError;
                    currError = target - distanceTravelledR;
                    accumError += currError;
                    errorRate = (prevError - currError) / (float)currDt;
                    PIDValue = kP * currError + kI * accumError + kD * errorRate;
                    PIDValue = Range.clip(PIDValue, -maxPower, maxPower);
                    driveRPower = PIDValue;
                    if(currError < acceptableError) {
                        power = 0.0f;
                        isMoving = false;
                    }
                    Thread.sleep(10);
                    // add in fine tune mode later
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }*/
}
