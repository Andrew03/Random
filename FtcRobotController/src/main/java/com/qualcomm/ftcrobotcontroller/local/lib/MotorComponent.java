package com.qualcomm.ftcrobotcontroller.local.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public abstract class MotorComponent {
    private int ticksPerRevolution = 1120;
    private int wheelDiameter = 1;
    private int gearRatio = 1;
    DcMotor motors[];
    enum Direction {
        FORWARD,
        REVERSE
    } Direction direction = Direction.FORWARD;
    abstract void update();
    void setWheelDiameter(int wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }
    void setGearRatio(int gearRatio) {
        this.gearRatio = gearRatio;
    }
    void setDirection(Direction direction) {
        this.direction = direction;
    }
    void setMotors(DcMotor ... motors) {
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
    }
    int getWheelDiameter() {
        return  wheelDiameter;
    }
    int getGearRatio() {
        return gearRatio;
    }
    Direction getDirection() {
        return direction;
    }
    DcMotor[] getMotors() {
        return motors;
    }
}
