package com.qualcomm.ftcrobotcontroller.local.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Anjali on 12/22/2015.
 */

public class ScrubTest extends OpMode {

    DcMotor M_driveBR, M_driveBL, M_driveFR, M_driveFL;
    final double C_STICK_TOP_THRESHOLD = 0.90d;
    final double STOP = 0.0d;
    final double    DRIVE_POWER = 0.8d,
                    TURN_POWER  = 0.8d;
    final double TURN_THRESHOLD = 3.0d;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }
    private Quadrant findQuadrant(Gamepad gamepad, String stick) {
        if(stick.toLowerCase().equals("left")) {
            if (gamepad.left_stick_y != 0.0d && gamepad.left_stick_x != 0.0d) {
                if (-gamepad.left_stick_y >= 0.0d) {
                    if (gamepad.left_stick_x >= 0.0d) {
                        return Quadrant.Q1;
                    } else {
                        return Quadrant.Q2;
                    }
                } else {
                    if (gamepad.left_stick_x >= 0.0d) {
                        return Quadrant.Q3;
                    } else {
                        return Quadrant.Q4;
                    }
                }
            } else {
                return Quadrant.NA;
            }
        } else {
            if (gamepad.right_stick_y != 0.0d && gamepad.right_stick_x != 0.0d) {
                if (-gamepad.right_stick_y >= 0.0d) {
                    if (gamepad.right_stick_x >= 0.0d) {
                        return Quadrant.Q1;
                    } else {
                        return Quadrant.Q2;
                    }
                } else {
                    if (gamepad.right_stick_x >= 0.0d) {
                        return Quadrant.Q3;
                    } else {
                        return Quadrant.Q4;
                    }
                }
            } else {
                return Quadrant.NA;
            }
        }
    }
    private double findAngle() {
        final double TICKS_TO_DEGREES = 0.0d;
        return (((M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2.0d) - ((M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2.0d)) * TICKS_TO_DEGREES % 360.0d;
    }
    private void turn(double angle) {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double kP = 0.000d;
        double kI;
        double accumError = 0.0d;
        double thresholdPower = 0.1d;
        double error = findAngle() - angle;
        accumError += error;
        double PIDValue = error * kP;
        if(Math.abs(error) <= TURN_THRESHOLD) {
            M_drivePowerR = STOP;
            M_drivePowerL = STOP;
        } else {
            PIDValue = Range.clip(PIDValue, -1.0d, 1.0d);
            M_drivePowerR = PIDValue;
            M_drivePowerL = -PIDValue;
        }
    }
    double M_drivePowerR = STOP, M_drivePowerL = STOP;
    double targetAngleL, targetAngleR;
    enum DriveMode{
        TANK,
        BUTTON,
        THRUST,
        ARCADE
    } DriveMode driveMode = DriveMode.TANK;
    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4,
        NA
    }

    @Override
    public void init() {
        M_driveFR = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        /*
        if(gamepad1.dpad_up) {
            driveMode = DriveMode.TANK;
        } else if(gamepad1.dpad_left) {
            driveMode = DriveMode.BUTTON;
        } else if(gamepad1.dpad_down) {
            driveMode = DriveMode.THRUST;
        } else if(gamepad1.dpad_right) {
            driveMode = DriveMode.ARCADE;
        }

        switch (driveMode) {
            case TANK:
                M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                M_drivePowerL = convertStick(-gamepad1.left_stick_y);
                telemetry.addData("Drive Mode: ", "Tank");
                break;
            case BUTTON:
                // drive forward
                if(gamepad1.y) {
                    M_drivePowerR = DRIVE_POWER;
                    M_drivePowerL = DRIVE_POWER;
                // drive backward
                } else if(gamepad1.a) {
                    M_drivePowerR = -DRIVE_POWER;
                    M_drivePowerL = -DRIVE_POWER;
                // turn to the right
                } else if(gamepad1.b) {
                    M_drivePowerR = -TURN_POWER;
                    M_drivePowerL = TURN_POWER;
                // turn to the left
                } else if(gamepad1.x) {
                    M_drivePowerR = TURN_POWER;
                    M_drivePowerL = -TURN_POWER;
                }
                telemetry.addData("Drive Mode: ", "Button");
                break;
            case THRUST:
                Quadrant quadrant = findQuadrant(gamepad1, "left");
                switch (quadrant) {
                    case Q1:
                    case Q4:
                        targetAngleL = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    case Q2:
                    case Q3:
                        targetAngleL = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    default:
                        targetAngleL = findAngle();
                        break;
                }
                if(Math.abs(targetAngleL - findAngle()) > TURN_THRESHOLD) {
                    turn(targetAngleL);
                } else {
                    M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                    M_drivePowerL = convertStick(-gamepad1.right_stick_y);
                }
                telemetry.addData("Drive Mode: ", "Thrust");
                break;
            case ARCADE:
                Quadrant quadrantL = findQuadrant(gamepad1, "left");
                switch (quadrantL) {
                    case Q1:
                    case Q4:
                        targetAngleL = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    case Q2:
                    case Q3:
                        targetAngleL = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    default:
                        targetAngleL = findAngle();
                        break;
                }
                if(Math.abs(targetAngleL - findAngle()) > TURN_THRESHOLD) {
                    turn(targetAngleL);
                } else {
                    M_drivePowerR = convertStick(-gamepad1.left_stick_y / gamepad1.left_stick_x);
                    M_drivePowerL = convertStick(-gamepad1.left_stick_y / gamepad1.left_stick_x);
                }
                if(M_drivePowerR == 0.0d && M_drivePowerL == 0.0d) {
                    Quadrant quadrantR = findQuadrant(gamepad1, "right");
                    switch (quadrantR) {
                        case Q1:
                        case Q4:
                            targetAngleR = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                            break;
                        case Q2:
                        case Q3:
                            targetAngleR = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                            break;
                        default:
                            targetAngleR = findAngle();
                            break;
                    }
                    turn(targetAngleR);
                }
                telemetry.addData("Drive Mode: ", "Arcade");
                break;
            default:
                break;
        }*/

        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);

        M_driveFR.setPower(M_drivePowerR);
        M_driveFL.setPower(M_drivePowerL);
        M_driveBR.setPower(M_drivePowerR);
        M_driveBL.setPower(M_drivePowerL);
    }

    @Override
    public void stop() {
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
    }
}
