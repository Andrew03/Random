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
            S_basketTilt            = null, // front left servo of the pickup
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
                    S_BASKET_TILT_START_POS             = Servo.MAX_POSITION,
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
                    S_BASKET_RELEASE_END_POS            = Servo.MIN_POSITION,
                    S_BASKET_TILT_END_POS               = Servo.MIN_POSITION,
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
    double[] drivePowers;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_liftPosR               = S_LIFT_START_POS_R,
            S_liftPosL               = S_LIFT_START_POS_L,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_basketTiltPos          = S_BASKET_TILT_START_POS,
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

    // function necessity delcarations
    int[] motorTargetsDrive;
    int[] motorTargetsTurn;

    ElapsedTime clock;


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
        this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");
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

        motorTargetsDrive = new int[2];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);
        Arrays.fill(motorTargetsTurn, 0);

        clock = new ElapsedTime();

        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        double case1Time = 0;
        boolean hasBeenSet = false;
        boolean finished = false;
        drivePowers = new double[2];
        Arrays.fill(drivePowers, 0.0d);
        waitForStart();
        clock.startTime();
        while(opModeIsActive()) {

            switch (counter) {
                case 0:
                    if(!hasBeenSet) {
                    motorTargetsDrive = setDriveTarget(7.5d);
                    hasBeenSet = true;
                }
                finished = driveForward();
                if(finished) {
                    hasBeenSet = false;
                    counter++;
                    stopDriving();
                    telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                    telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                    telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                    telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                    sleep(800);
                }
                break;

                case 1:
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(72.0d);
                        hasBeenSet = true;
                    }
                    finished = turnRight();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                        sleep(800);
                    }
                    break;
                case 2:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(35.0d);
                        hasBeenSet = true;
                    }
                    finished = driveForward();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                        sleep(1200);
                    }
                    break;
                case 3:
                    M_driveFR.setPower(-0.25d);
                    M_driveFL.setPower(0.25d);
                    M_driveBR.setPower(-0.25d);
                    M_driveBL.setPower(0.25d);
                    sleep(500);
                    counter++;
                    break;
                    /*if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(20.0d);
                        hasBeenSet = true;
                    }
                    finished = turnRight();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                        sleep(800);
                    }*/
                case 4:
                    if(!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(12.5d);
                        hasBeenSet = true;
                    }
                    finished = driveForward();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                        sleep(800);
                    }

                default:
                    drivePowers[0] = STOP;
                    drivePowers[1] = STOP;
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
            M_drivePowerR = drivePowers[0];
            M_drivePowerL = drivePowers[1];
            M_driveFR.setPower(M_drivePowerR);
            M_driveFL.setPower(M_drivePowerL);
            M_driveBR.setPower(M_drivePowerR);
            M_driveBL.setPower(M_drivePowerL);
            telemetry.addData("Counter", counter);
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
        final double INCHES_TO_TICKS = 200.0d;
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        int[] targets = new int[2];
        targets[0] = (int)(motors[0].getCurrentPosition() + inches * INCHES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + inches * INCHES_TO_TICKS);
        return targets;
    }

    public boolean driveForward() {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.003d;
        double kI;
        double[] actualPIDValue = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            int error = motorTargetsDrive[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            //accumError[i] += error;
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
        for(int i = 0; i < 2; i++) {
            if(Math.abs(drivePowers[i]) > thresholdPower) {
                return false;
            }
        }
        return true;
    }

    public int[] setTurnTarget(double degrees) {
        final double DEGREES_TO_TICKS = 11.1d;
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
        double kP = 0.003d;
        double kI;
        double actualPIDValue[] = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            //int error = (int)(motorTargetsTurn[i] - (motors[i].getCurrentPosition() - motors[i + 2].getCurrentPosition()) / 2);
            int error = (int)(motorTargetsTurn[i] - motors[i].getCurrentPosition());
            PIDValue[i] = kP * error;
            //accumError[i] += error;
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
        for(int i = 0; i < 2; i++) {
            if(Math.abs(drivePowers[i]) > 0.05) {
                return false;
            }
        }
        return true;
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
