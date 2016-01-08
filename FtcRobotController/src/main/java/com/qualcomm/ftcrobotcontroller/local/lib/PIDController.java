package com.qualcomm.ftcrobotcontroller.local.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public class PIDController {
    public PIDController(int wheelDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, double conversionFactor, DcMotor ... motors) {
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
        this.threshold = threshold;
        this.slowDownStart = slowDownStart;
        this.fineTuneStart = fineTuneStart;
        this.powerMin = powerMin;
        this.conversionFactor = conversionFactor;
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
    }
    // variables that change depending on part of robot
    private int wheelDiameter;
    private int gearRatio;
    private int threshold;
    private double slowDownStart;
    private double fineTuneStart;
    private double powerMin;
    private double conversionFactor;
    private DcMotor[] motors;

    // loop this
    private double power = (Math.abs(error) < slowDownStart) ? kFast * error : kSlow * (tickOffset + error);
    /*if(Math.abs(error) < threshold) {
        power = 0.0d;
        break;
    }*/
    void run(double target) {
        final int TICKS_PER_REVOLUTION   = 1120;
        final double K_FAST              = 1 / (slowDownStart * TICKS_PER_REVOLUTION);
        final double K_SLOW              = (powerMin * slowDownStart) / (fineTuneStart * fineTuneStart * TICKS_PER_REVOLUTION);
        final double TICK_OFFSET         = (fineTuneStart * fineTuneStart * TICKS_PER_REVOLUTION) / slowDownStart;

        double targetInTicks            = target * conversionFactor;
        double currVal                  = 0;
        double error                    = 0;
        boolean hasReachedDestination = false;

        while(!hasReachedDestination) {
            for(int i = 0; i < Math.ceil(motors.length / 2); i++) {

            }
        }
    }
}
