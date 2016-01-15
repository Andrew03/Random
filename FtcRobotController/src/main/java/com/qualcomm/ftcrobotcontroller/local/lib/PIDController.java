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

    void run(double target) {
        final int TICKS_PER_REVOLUTION   = 1120;
        final double K_FAST              = 1 / (slowDownStart * TICKS_PER_REVOLUTION);
        final double K_SLOW              = (powerMin * slowDownStart) / (fineTuneStart * fineTuneStart * TICKS_PER_REVOLUTION);
        final double TICK_OFFSET         = (fineTuneStart * fineTuneStart * TICKS_PER_REVOLUTION) / slowDownStart;
        boolean isSymmetrical = (motors.length % 2 == 0);
        int sides;
        if(isSymmetrical) {
            sides = 1;
        } else {
            sides = 2;
        }
        int[] currVal       = new int[sides];
        int[] error         = new int[sides];
        int[] targetInTicks = new int[sides];
        double[] power      = new double[sides];
        for(int i = 0; i < sides; i++) {
            int currPos = 0;
            for(int j = 0; j < motors.length / sides; j++) {
                currPos += motors[j].getCurrentPosition();
            }
            targetInTicks[i] = (int)(target * conversionFactor) + (int)(currPos / (motors.length / sides));
        }
        boolean hasReachedDestination = false;

        while(!hasReachedDestination) {
            for(int i = 0; i < sides; i++) {
                int sidePos = 0;
                for(int j = 0; j < motors.length / sides; j++) {
                    sidePos += motors[i * j].getCurrentPosition();
                }
                currVal[i] = sidePos / (int)(sidePos / (motors.length / sides));
                error[i] = targetInTicks[i] - currVal[i];
                power[i] = (Math.abs(error[i]) > fineTuneStart) ? K_FAST * error[i] : K_SLOW * (TICK_OFFSET + error[i]);
                for(int j = 0; j < motors.length / sides; j++) {
                    motors[i * j].setPower(power[i]);
                }
                hasReachedDestination = (Math.abs(error[i]) < threshold);
            }
        }
        stop();
    }
    void stop() {
        for(DcMotor motor : motors) {
            motor.setPower(0.0d);
        }
    }
}
