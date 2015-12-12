package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Andrew on 10/23/2015.
 */

// instructions: //////////////////////////
    /*
    left and right joysticks gamepad 1 control driving, up down, tank drive
    right bumper to pick up, left bumper to push out from pickup
    right trigger to raise lift, left trigger to lower
    a to release balls, b to reset basket drop

    gamepad 2
    start selects basket control mode
    in auto:
        pressing b puts basket in right position
        pressing x puts basket in left position
        pressing a puts basket in down position
        holding b will rotate basket to the right
        holding a will rotate basket to the left
    in manual:
        holding b will rotate basket to the right
        holding x will rotate basket to the left
        holding a will tilt basket to the right
        holding y will tilt basket to the left
    up on dpad will extend both climber knockdowns
    down on dpad will retract both climber knockdowns
    left on dpad will extend left climber knockdown
    right on dpad will extend right climber knockdown


    fix on the robot
        lift doesn't go down all the way
        basket doesn't have a thing to stop balls from falling out
        guards on bottom to prevent balls from going in
        gaurds to prevent balls from going around isnide basket
        adhesive doesn't stick
        I don't have any decent auton so use nullOpMode if they make you select one
        run teleopmode for teleop
     */

// 1120 pulses per rev

// the main teleop opmode we will be using
public class TeleOp extends OpMode {

    public TeleOp() {

    }
    // motor declarations
    DcMotor M_driveFR = null, // front right drive motor
            M_driveFL = null, // front left drive motor
            M_driveBR = null, // back right drive motor
            M_driveBL = null, // back left drive motor
            M_pickup  = null, // pickup motor
            M_lift    = null; // lift motor

    // servo declarations
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDepositRotate = null, // servo that rotates climbers
            S_climbersDepositDrop   = null, // servo that drops climbers
            S_basketRotate          = null, // servo that controls basket rotation
            S_basketRelease         = null, // servo that releases blocks
            S_basketTilt            = null; // servo that controls tilt

    // all of the important constants
    final double    STOP                    = 0.0d,
                    MAX_POWER               = 1.0d;

    // all of the constant motor powers
    final double    PICKUP_POWER         = 0.65d,
                    LIFT_POWER_FAST      = 1.0d,
                    LIFT_POWER_SLOW      = 1.0d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MAX_POSITION,
                    S_CLIMBERS_DEPOSIT_ROTATE_START_POS = Servo.MIN_POSITION,
                    S_climbers_DEPOSIT_DROP_START_POS   = 0.0d,
                    S_BASKET_TILT_START_POS             = 0.875d,
                    S_BASKET_RELEASE_START_POS          = 0.34d;

    // all of the ending/close servo positions
    final double    S_CLIMBERS_KNOCKDOWN_END_POS_R      = 0.494d,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_ROTATE_END_POS   = 0.549d,
                    S_climbers_DEPOSIT_DROP_END_POS     = 0.0d,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION;

    // special pos for tilt servo
    final double    S_BASKET_TILT_POS_RIGHT     = 0.290d,
                    S_BASKET_TILT_POS_LEFT      = Servo.MAX_POSITION;

    // motor powers
    double  M_drivePowerR = STOP,
             M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP;

    // servo positions
    double  S_climbersKnockdownPosR     = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL     = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositRotatePos  = S_CLIMBERS_DEPOSIT_ROTATE_START_POS,
            S_climbersDepositDropPos    = S_climbers_DEPOSIT_DROP_START_POS,
            S_basketTiltPos             = S_BASKET_TILT_START_POS,
            S_basketReleasePos          = S_BASKET_RELEASE_START_POS;

    // servo powers
    final double    S_SPEED_STOP                        = 0.5d,
                    S_BASKET_ROTATE_SPEED_LEFT_SLOW     = 0.56d,
                    S_BASKET_ROTATE_SPEED_LEFT_FAST     = 0.58d,
                    S_BASKET_ROTATE_SPEED_RIGHT_SLOW    = 0.47d,
                    S_BASKET_ROTATE_SPEED_RIGHT_FAST    = 0.45d;

    // servo speeds
    double  S_basketRotateSpeed = S_SPEED_STOP;

    enum BasketMode {
        AUTO,
        MANUAL
    } BasketMode basketMode = BasketMode.AUTO;

    enum P1Mode {
        P1,
        P2
    } P1Mode p1Mode = P1Mode.P1;

    boolean isP1Tilt    = false,
            isP1Rotate  = false;
    boolean isDepositSet = false;

    private final float C_STICK_TOP_THRESHOLD = 0.85f;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

    // maps motor variables to their hardware counterparts
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
        S_climbersKnockdownR    = hardwareMap.servo.get("S_climbersKnockdownR");
        S_climbersKnockdownL    = hardwareMap.servo.get("S_climbersKnockdownL");
        S_climbersDepositRotate = hardwareMap.servo.get("S_climbersDepositRotate");
        S_climbersDepositDrop   = hardwareMap.servo.get("S_climbersDepositDrop");
        S_basketRotate          = hardwareMap.servo.get("S_basketRotate");
        S_basketRelease         = hardwareMap.servo.get("S_basketRelease");
        S_basketTilt            = hardwareMap.servo.get("S_basketTilt");
    }

    // configures the motors to desired configurations
    private void configureMotors() {
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        M_pickup.setDirection(DcMotor.Direction.REVERSE);
        M_lift.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init() {
        // mapping motor variables to their hardware counterparts
        setMotors();
        // mapping servo variables to their hardware counterparts
        setServos();
        // fixing motor directions
        configureMotors();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        ///////////////////////////////////////// Controller 1 controls ///////////////////////////////////////////////
        // driving control block
        if(gamepad1.right_trigger > 0.0d) {
            M_drivePowerR = gamepad1.right_trigger;
            M_drivePowerL = gamepad1.right_trigger;
        } else if(gamepad1.left_trigger > 0.0d) {
            M_drivePowerR = -gamepad1.left_trigger;
            M_drivePowerL = -gamepad1.left_trigger;
        } else {
            M_drivePowerR = STOP;
            M_drivePowerL = STOP;
        }
        if(gamepad1.b) {
            M_drivePowerR *= -1.0d;
        } else if(gamepad1.x) {
            M_drivePowerL *= -1.0d;
        }

        // basket control block
        if(gamepad1.dpad_right) {
            S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_RIGHT_SLOW;
            isP1Rotate = true;
        } else if(gamepad1.dpad_left) {
            S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_LEFT_SLOW;
            isP1Rotate = true;
        } else {
            S_basketRotateSpeed = S_SPEED_STOP;
            isP1Rotate = false;
        }
        if(gamepad1.dpad_up && S_basketTiltPos < 0.98d) {
            S_basketTiltPos += 0.01d;
            isP1Tilt = true;
        } else if(gamepad1.dpad_down && S_basketTiltPos > 0.02d) {
            S_basketTiltPos -= 0.01d;
            isP1Tilt = true;
        } else {
            isP1Tilt = false;
        }

        // basket reset block
        if(gamepad1.start) {
            S_basketTiltPos = S_BASKET_TILT_START_POS;
            S_basketRotateSpeed = S_SPEED_STOP;
        }

        // selecting player 1 mode
        if(gamepad1.right_stick_y > 0.5d) {
            p1Mode = P1Mode.P1;
        } else if(gamepad1.left_stick_y > 0.5d) {
            p1Mode = P1Mode.P2;
        }

        ///////////////////////////////////////// Controller 2 controls ///////////////////////////////////////////////
        // climber knockdown control block
        if(gamepad2.dpad_right) {
            S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_END_POS_R;
        }
        if(gamepad2.dpad_left) {
            S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
        }
        if(gamepad2.dpad_up) {
            S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_END_POS_R;
            S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
        }else if(gamepad2.dpad_down) {
            S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_START_POS_R;
            S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_START_POS_L;
        }

        // toggle basket control mode
        if(gamepad2.right_stick_y > 0.5d) {
            basketMode = BasketMode.MANUAL;
        } else if(gamepad2.left_stick_y > 0.5d) {
            basketMode = BasketMode.AUTO;
        }

        // basket control block
        switch (basketMode) {
            case AUTO:
                if(!isP1Rotate && !isP1Tilt) {
                    if (gamepad2.b) {
                        S_basketTiltPos = S_BASKET_TILT_POS_RIGHT;
                        S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_RIGHT_FAST;
                    } else if (gamepad2.x) {
                        S_basketTiltPos = S_BASKET_TILT_POS_LEFT;
                        S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_LEFT_FAST;
                    } else if (gamepad2.a) {
                        S_basketTiltPos = S_BASKET_TILT_START_POS;
                        S_basketRotateSpeed = S_SPEED_STOP;
                    } else {
                        S_basketRotateSpeed = S_SPEED_STOP;
                    }
                }
                break;
            case MANUAL:
                if(!isP1Rotate) {
                    if (gamepad2.b) {
                        S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_RIGHT_SLOW;
                    } else if (gamepad2.x) {
                        S_basketRotateSpeed = S_BASKET_ROTATE_SPEED_LEFT_SLOW;
                    } else {
                        S_basketRotateSpeed = S_SPEED_STOP;
                    }
                }
                if(!isP1Tilt) {
                    if (gamepad2.y && S_basketTiltPos < 0.98d) {
                        S_basketTiltPos += 0.01d;
                    } else if (gamepad2.a && S_basketTiltPos > 0.02d) {
                        S_basketTiltPos -= 0.01d;
                    }
                }
                break;
            default:
                break;
        }

        ///////////////////////////////////// variable commands /////////////////////////////////
        switch (p1Mode) {
            //////////////////////////////// P1 mode commands ////////////////////////////////////////////
            case P1:
                // pickup control block
                if (gamepad1.right_bumper) {
                    M_pickupPower = PICKUP_POWER;
                } else if (gamepad1.left_bumper) {
                    M_pickupPower = -PICKUP_POWER;
                } else {
                    M_pickupPower = STOP;
                }

                // basket release control block
                if(gamepad1.y) {
                    S_basketReleasePos = S_BASKET_RELEASE_END_POS;
                } else if(gamepad1.a) {
                    S_basketReleasePos = S_BASKET_RELEASE_START_POS;
                }

                // lift control block
                if(gamepad2.right_trigger > 0.0d) {
                    M_liftPower = LIFT_POWER_FAST;
                } else if(gamepad2.left_trigger > 0.0d) {
                    M_liftPower = -LIFT_POWER_FAST;
                } else if(gamepad2.right_bumper) {
                    M_liftPower = LIFT_POWER_SLOW;
                } else if(gamepad2.left_bumper) {
                    M_liftPower = -LIFT_POWER_SLOW;
                } else {
                    M_liftPower = STOP;
                }

                break;
            //////////////////////////////// P2 mode commands ////////////////////////////////////////////
            case P2:
                // lift control block
                if(gamepad1.right_bumper) {
                    M_liftPower = LIFT_POWER_SLOW;
                } else if(gamepad1.left_bumper) {
                    M_liftPower = -LIFT_POWER_SLOW;
                }
                // basket release control block
                if(gamepad1.y) {
                    if(!isDepositSet) {
                        S_climbersDepositRotatePos = S_CLIMBERS_DEPOSIT_ROTATE_END_POS;
                        isDepositSet = true;
                    } else {
                        S_climbersDepositDropPos = S_climbers_DEPOSIT_DROP_END_POS;
                    }
                } else if(gamepad1.a) {
                    S_climbersDepositDropPos = S_climbers_DEPOSIT_DROP_START_POS;
                    S_climbersDepositRotatePos = S_CLIMBERS_DEPOSIT_ROTATE_START_POS;
                    isDepositSet = false;
                }
                break;
            default:
                break;
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
        S_climbersDepositRotate.setPosition(S_climbersDepositRotatePos);
        S_climbersDepositDrop.setPosition(S_climbersDepositDropPos);
        S_basketRotate.setPosition(S_basketRotateSpeed);
        S_basketRelease.setPosition(S_basketReleasePos);
        S_basketTilt.setPosition(S_basketTiltPos);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Control mode", basketMode);

    }

    @Override
    public void stop() {
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_pickup.setPower(STOP);
        M_lift.setPower(STOP);

        S_climbersKnockdownR.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_R);
        S_climbersKnockdownL.setPosition(S_CLIMBERS_KNOCKDOWN_START_POS_L);
        S_climbersDepositRotate.setPosition(S_CLIMBERS_DEPOSIT_ROTATE_START_POS);
        S_climbersDepositDrop.setPosition(S_climbers_DEPOSIT_DROP_START_POS);
        S_basketRotate.setPosition(S_SPEED_STOP);
        S_basketRelease.setPosition(S_BASKET_RELEASE_START_POS);
        S_basketTilt.setPosition(S_BASKET_TILT_START_POS);
    }

    /*private class ControllerThread implements Runnable {
        private final float C_STICK_TOP_THRESHOLD = 0.85f,      // least value for which stick value read from motor will be 1.0f
                            PICKUP_POWER = 1.0f,
                            LIFT_POWER = 1.0f;

        // converts all of the controller sticks into more sensitive values
        // use a negative value for y axis since controller reads -1 when pushed forward
        private float convertStick(float controllerValue) {   return (float) Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

        // the main loop function
        public void run() {
            try {
                while(!Thread.currentThread().isInterrupted()) {
                    driveRPower = convertStick(-gamepad1.right_stick_y);
                    driveLPower = convertStick(-gamepad1.left_stick_y);

                    if (gamepad1.right_bumper) {
                        pickupPower = PICKUP_POWER;
                    } else if (gamepad1.left_bumper) {
                        pickupPower = -PICKUP_POWER;
                    } else {
                        pickupPower = STOP;
                    }

                    if(gamepad1.right_trigger > 0.0f) {
                        liftPower = LIFT_POWER;
                    } else if(gamepad1.left_trigger > 0.0f) {
                        liftPower = -LIFT_POWER;
                    } else {
                        liftPower = STOP;
                    }

                    telemetry.addData("Thread is running", "Thread is running");
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

    }*/
}

