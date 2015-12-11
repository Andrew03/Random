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
            M_lift      = null; // lift motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_basketRotate          = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_basketTilt            = null, // front left servo of the pickup
            S_buttonPusher          = null;
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
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_BASKET_ROTATE_START_POS           = 0.37d,
                    S_BASKET_TILT_START_POS             = 0.875d,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
                    S_BUTTON_PUSHER_START_POS           = Servo.MIN_POSITION;


    // all of the ending/close servo positions
    final double    S_CLIMBERS_KNOCKDOWN_END_POS_R      = 0.494d,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_BUTTON_PUSHER_END_POS             = 0.141d;

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
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_basketTiltPos          = S_BASKET_TILT_START_POS,
            S_buttonPusherPos        = S_BUTTON_PUSHER_START_POS;

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
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        this.S_basketRotate         = this.hardwareMap.servo.get("S_basketRotate");
        this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");

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
                        while(waitingForClick()) {
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                            sleep(100);
                        }
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
                        while(waitingForClick()) {
                        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                            sleep(100);
                        }
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
                        while(waitingForClick()) {
                            telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                            telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                            telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                            telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                            sleep(100);
                        }
                    }
                    break;
                case 3:
                    // failed here last time
                    if(!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(20.0d);
                        hasBeenSet = true;
                    }
                    finished = turnRight();
                    if(finished) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        while(waitingForClick()) {
                            telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
                            telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
                            telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
                            telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
                            sleep(100);
                        }
                    }
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
        final double DEGREES_TO_TICKS = 12.2d;
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
