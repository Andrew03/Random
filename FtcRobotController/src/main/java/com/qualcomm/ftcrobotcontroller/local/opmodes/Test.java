package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import com.qualcomm.ftcrobotcontroller.local.lib.LiftPIDThread;
import com.qualcomm.ftcrobotcontroller.local.lib.DrivePIDThread;
import com.qualcomm.ftcrobotcontroller.local.lib.TurnPIDThread;

/**
 * Created by Andrew on 11/20/2015.
 */
public class Test extends OpMode {
    // motor declarations
    DcMotor M_driveFR = null, // front right drive motor
            M_driveFL = null, // front left drive motor
            M_driveBR = null, // back right drive motor
            M_driveBL = null, // back left drive motor
            M_pickup = null, // pickup motor
            M_lift = null; // lift motor

    // servo declarations
    Servo S_climbersKnockdownR = null, // right servo that knocks down climbers
            S_climbersKnockdownL = null, // left servo that knocks down climbers
            S_climbersDeposit = null, // servo that deposits climbers
            S_basketRotate = null, // right servo on the basket
            S_basketRelease = null, // left servo on the basket
            S_basketTilt = null; // front left servo of the pickup

    // sensor declarations
    ColorSensor colorSensor = null; // color sensor

    // all of the important constants
    final double    STOP = 0.0d,
                    MAX_POWER = 1.0d;
    final int       TICKS_PER_REVOLUTION = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER = 0.65d,
                    LIFT_POWER = 1.0d,
                    CLAMP_POWER = 0.5d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MAX_POSITION,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.635d,
                    S_BASKET_ROTATE_START_POS           = 0.37d,
                    S_BASKET_TILT_START_POS             = 0.18d,
                    S_BASKET_RELEASE_START_POS          = 0.34d,
                    S_BUTTON_PUSHER_START_POS           = Servo.MIN_POSITION;


    // all of the ending/close servo positions
    final double    S_CLIMBERS_KNOCKDOWN_END_POS_R     = 0.494d,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_BASKET_ROTATE_END_POS             = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_BUTTON_PUSHER_END_POS             = Servo.MIN_POSITION;

    // special pos for tilt servo
    final double    S_BASKET_TILT_POS_RIGHT     = Servo.MIN_POSITION,
                    S_BASKET_TILT_POS_LEFT      = Servo.MAX_POSITION,
                    S_BASKET_ROTATE_POS_RIGHT   = Servo.MAX_POSITION,
                    S_BASKET_ROTATE_POS_LEFT    = Servo.MIN_POSITION;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_basketTiltPos          = S_BASKET_TILT_START_POS,
            S_basketReleasePos       = S_BASKET_RELEASE_START_POS,
            S_basketRotatePos        = S_BASKET_ROTATE_START_POS,
            S_buttonPusherPos        = S_BUTTON_PUSHER_START_POS;

    // servo powers
    final double    S_BASKET_TILT_SPEED_LEFT    = 0.75d,
                    S_BASKET_TILT_SPEED_RIGHT   = 0.25d,
                    S_BASKET_TILT_SPEED_DOWN    = 0.55d,
                    S_SPEED_STOP                = 0.5d,
                    S_BASKET_ROTATE_SPEED_LEFT  = 0.55d,
                    S_BASKET_ROTATE_SPEED_RIGHT = 0.45d,
                    S_CLIMBERS_DEPOSIT_SPEED    = 0.02d;

    // servo speeds
    double  S_basketTiltSpeed = S_SPEED_STOP,
            S_basketRotateSpeed = S_SPEED_STOP;

    // PID Threads
    LiftPIDThread liftPIDThread;
    DrivePIDThread drivePIDThread;
    TurnPIDThread turnPIDThread;

    // PID Threads Constants
    final double    liftKP  = 0.0d,
                    liftKI  = 0.0d,
                    liftKD  = 0.0d;
    final double    driveKP = 0.0d,
                    driveKD = 0.0d,
                    driveKI = 0.0d;
    final double    turnKP  = 0.003d, // orig 0.007 for 60, 0.0045 for 40
                    turnKI  = 0,//0.000025d,
                    turnKD  = 0.0d;

    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4
    } Quadrant quadrant;

    enum BasketMode {
        AUTO,
        MANUAL
    } BasketMode basketMode = BasketMode.AUTO;

    enum KnockDownPos {
        IN,
        OUT
    } KnockDownPos  knockDownPosR = KnockDownPos.IN,
                    knockDownPosL = KnockDownPos.IN;

    private final float C_STICK_TOP_THRESHOLD = 0.85f;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }
    private boolean isRed() {   return colorSensor.red() > colorSensor.blue();  }
    private boolean isBlue() {  return colorSensor.blue() > colorSensor.red();  }

    private double setServoSpeed(Servo servo, double targetPos, double power) {
        if (servo.getPosition() > targetPos) {
            power -= 0.01d;
        } else {
            power += 0.01d;
        }
        return Range.clip(power, 0.0d, 1.0d);
    }

    // maps motor varaibles to their hardware counterparts
    private void setMotors() {
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");
    }

    // maps servo variables to their hardware counterparts
    private void setServos() {
        S_climbersKnockdownR   = hardwareMap.servo.get("S_climbersKnockdownR");
        S_climbersKnockdownL   = hardwareMap.servo.get("S_climbersKnockdownL");
        S_climbersDeposit      = hardwareMap.servo.get("S_climbersDeposit");
        S_basketRotate         = hardwareMap.servo.get("S_basketRotate");
        S_basketRelease        = hardwareMap.servo.get("S_basketRelease");
        S_basketTilt           = hardwareMap.servo.get("S_basketTilt");
    }

    // maps sensors to their hardware counterparts
    private void setSensors() {
        colorSensor            = hardwareMap.colorSensor.get("S_colorSensor");
    }

    // configures the motors to desired configurations
    private void configureMotors() {
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        M_pickup.setDirection(DcMotor.Direction.REVERSE);
        M_lift.setDirection(DcMotor.Direction.REVERSE);
    }

    // prepares the encoders
    private void setEncoders() {
        // resets encoder values
        M_driveFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        M_driveFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        M_driveBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        M_driveBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        M_lift.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // sets encoders up for reading
        M_driveFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        M_driveFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        M_driveBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        M_driveBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        M_lift.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    @Override
    public void init() {
        // mapping motor variables to their hardware counterparts
        setMotors();
        // mapping servo variables to their hardware counterparts
        setServos();
        // mapping sensors to their hardware counterparts
        setSensors();
        // fixing motor directions
        configureMotors();
        // prepares for encoder usage
        setEncoders();
    }

    @Override
    public void start() {

        // defining PID Threads
        liftPIDThread = new LiftPIDThread(liftKP, liftKI, liftKD, this.M_lift);
        drivePIDThread = new DrivePIDThread(driveKP, driveKI, driveKD, this.M_driveFR, this.M_driveFL, this.M_driveBR, this.M_driveBL);
        turnPIDThread = new TurnPIDThread(turnKP, turnKI, turnKD, this.M_driveFR, this.M_driveFL, this.M_driveBR, this.M_driveBL);

        // starting PID Threads
        //liftPIDThread.start();
        //drivePIDThread.start();
        //turnPIDThread.start();
        //turnPIDThread.setTarget(910.0d);

    }

    @Override
    public void loop() {
        /*double encoderValueRs = (M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2;
        double encoderValueLs = (M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2;
        double currentAngle = (encoderValueLs - encoderValueRs) / 2 * ticksToDegrees;
        double targetAngle;
        quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        switch(driveMode) {
            case TANK:
                M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                M_drivePowerL = convertStick(-gamepad1.left_stick_y);
                break;
            case FPS:
                break;
            case ARCADE:
                double turnThreshold = 0.25d;
                quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                currentAngle = (encoderValueLs - encoderValueRs) / 2 * ticksToDegrees;  // in degrees
                targetAngle = findTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, quadrantL); // in degrees
                if(Math.abs(targetAngle - currentAngle) > turnThreshold) {
                    // use TurnPID variables, make an if statement



                    M_drivePowerR = turnPIDVal;
                    M_drivePowerL = turnPIDVal;
                } else {
                    M_drivePowerR = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                    M_drivePowerL = M_drivePowerR;
                }
                break;
            case THRUST:
                quadrantL = findQuadrant(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                M_drivePowerR = gamepad1.right_stick_y;
                M_drivePowerL = M_drivePowerR;
                break;
            default:
                break;
        }
        */

        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);

        // pickup control block
        if (gamepad1.right_bumper) {
            M_pickupPower = PICKUP_POWER;
        } else if (gamepad1.left_bumper) {
            M_pickupPower = -PICKUP_POWER;
        } else {
            M_pickupPower = STOP;
        }

        // lift control block
        if(gamepad1.right_trigger > 0.0f) {
            M_liftPower = LIFT_POWER;
        } else if(gamepad1.left_trigger > 0.0f) {
            M_liftPower = -LIFT_POWER;
        } else {
            M_liftPower = STOP;
        }
        
        // basket release control block
        if(gamepad1.a) {
            S_basketReleasePos = S_BASKET_RELEASE_END_POS;
        } else if(gamepad1.b) {
            S_basketReleasePos = S_BASKET_RELEASE_START_POS;
        }

        // toggle basket control mode
        if(gamepad2.start) {
            if(basketMode == BasketMode.AUTO) {
                basketMode = BasketMode.MANUAL;
            } else {
                basketMode = BasketMode.AUTO;
            }
        }

        switch (basketMode) {
            case AUTO:
                // basket tilt control block
                if (gamepad2.b) {
                    S_basketTiltPos = S_BASKET_TILT_POS_RIGHT;
                    S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_BASKET_ROTATE_POS_RIGHT, S_basketRotateSpeed);
                    S_basketRotatePos = S_basketRotate.getPosition();
                } else if (gamepad2.x) {
                    S_basketTiltPos = S_BASKET_TILT_POS_LEFT;
                    S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_BASKET_ROTATE_POS_LEFT, S_basketRotateSpeed);
                    S_basketRotatePos = S_basketRotate.getPosition();
                } else if (gamepad2.a) {
                    S_basketTiltPos = S_BASKET_TILT_START_POS;
                    S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_BASKET_ROTATE_START_POS, S_basketRotateSpeed);
                    S_basketRotatePos = S_basketRotate.getPosition();
                } else {
                    S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_basketRotatePos, S_basketRotateSpeed);
                }
                break;
            case MANUAL:
                if(gamepad2.b) {
                    S_basketRotatePos += 0.01d;
                } else if(gamepad2.x) {
                    S_basketRotatePos -= 0.01d;
                }
                if(gamepad2.y && S_basketTiltPos < 0.98d) {
                    S_basketTiltPos += 0.01d;
                } else if(gamepad2.a && S_basketTiltPos > 0.02d) {
                    S_basketTiltPos -= 0.01d;
                }
                S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_basketRotatePos, S_basketRotateSpeed);
                break;
            default:
                break;
        }

        /*// basket rotate control block
        if(gamepad2.dpad_right) {
            S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_RIGHT;
            S_basketRotatePos = S_basketRotate.getPosition();
        } else if(gamepad2.dpad_left) {
            S_basketRotatePos = S_BASKET_ROTATE_SPEED_LEFT;
            S_basketRotatePos = S_basketRotate.getPosition();
        } else {
            S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_basketRotatePos, S_basketRotateSpeed);
        }
        if(gamepad2.start) {
            S_basketTiltPos = S_BASKET_TILT_START_POS;
            S_basketRotateSpeed = setServoSpeed(S_basketRotate, S_BASKET_ROTATE_START_POS, S_basketRotateSpeed);
        }*/

        // right climber knockdown control block
        if(gamepad2.dpad_right) {
            switch (knockDownPosR) {
                case IN:
                    S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_END_POS_R;
                    knockDownPosR = KnockDownPos.OUT;
                    break;
                case OUT:
                    S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_START_POS_R;
                    knockDownPosR = KnockDownPos.IN;
                    break;
                default:
                    break;
            }
        }

        // left climber knockdown control block
        if(gamepad2.dpad_left) {
            switch (knockDownPosL) {
                case IN:
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
                    knockDownPosL = KnockDownPos.OUT;
                    break;
                case OUT:
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_START_POS_L;
                    knockDownPosL = KnockDownPos.IN;
                    break;
                default:
                    break;
            }
        }

        // climber deposit pos
        if(gamepad2.dpad_up && S_climbersDepositPos > S_CLIMBERS_DEPOSIT_END_POS) {
            S_climbersDepositPos -= S_CLIMBERS_DEPOSIT_SPEED;
        } else if(gamepad2.dpad_down) {
            S_climbersDepositPos = S_CLIMBERS_DEPOSIT_START_POS;
        }

        // updates all the motor powers
        M_driveBR.setPower(M_drivePowerR);
        M_driveBL.setPower(M_drivePowerL);
        M_driveFR.setPower(M_drivePowerR);
        M_driveFL.setPower(M_drivePowerL);
        M_pickup.setPower(M_pickupPower);
        M_lift.setPower(M_liftPower);

        // updates all the servo positions
        S_climbersKnockdownR.setPosition(S_climbersKnockdownPosR);
        S_climbersKnockdownL.setPosition(S_climbersKnockdownPosL);
        S_climbersDeposit.setPosition(S_climbersDepositPos);
        S_basketRotate.setPosition(S_basketRotateSpeed);
        S_basketRelease.setPosition(S_basketReleasePos);
        S_basketTilt.setPosition(S_basketTiltPos);

        //S_test.setPosition(S_testPos);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Servo Rot Pos", S_basketRotate.getPosition());
        telemetry.addData("Servo Tilt Pos", S_basketTilt.getPosition());
        if(isRed()) {
            telemetry.addData("It's red!", ":D");
        } else if(isBlue()) {
            telemetry.addData("It's blue!", ":(");
        }
    }
    @Override
    public void stop() {
        //turnPIDThread.interrupt();

        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_pickup.setPower(STOP);
        M_lift.setPower(STOP);

        S_climbersKnockdownR.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_R);
        S_climbersKnockdownL.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_L);
        S_climbersDeposit.setPosition(S_CLIMBERS_DEPOSIT_START_POS);
        S_basketRotate.setPosition(S_SPEED_STOP);
        S_basketRelease.setPosition(S_BASKET_RELEASE_START_POS);
        S_basketTilt.setPosition(S_BASKET_TILT_START_POS);

    }
}
