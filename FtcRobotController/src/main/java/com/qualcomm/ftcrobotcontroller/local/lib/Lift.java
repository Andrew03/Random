package com.qualcomm.ftcrobotcontroller.local.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public class Lift extends MotorComponent {
    public Lift() {
        this(0, 0);
    }
    public Lift(int wheelDiameter, int gearRatio) {
        this(wheelDiameter, gearRatio, Direction.FORWARD);
    }
    public Lift(int wheelDiameter, int gearRatio, Direction direction) {
        setWheelDiameter(wheelDiameter);
        setGearRatio(gearRatio);
        setDirection(direction);
    }
    public void setMotors(DcMotor ... motors) {
        this.setMotors(motors);
    }
    void update() {

    }
}
