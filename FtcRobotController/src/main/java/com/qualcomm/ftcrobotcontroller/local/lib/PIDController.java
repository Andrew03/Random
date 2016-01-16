package com.qualcomm.ftcrobotcontroller.local.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public class PIDController {
    public PIDController(int wheelDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, TypePID typePID, DcMotor ... motors) {
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
        this.threshold = threshold;
        this.slowDownStart = slowDownStart;
        this.fineTuneStart = fineTuneStart;
        this.powerMin = powerMin;
        this.typePID = typePID;
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
        boolean isSymmetrical = (motors.length % 2 == 0);
        if(isSymmetrical) {
            sides = 1;
        } else {
            sides = 2;
        }
        targets             = new int[sides];
    }
    public enum TypePID {
        DRIVE,
        TURN,
        LIFT
    } TypePID typePID;
    // variables that change depending on part of robot
    private int wheelDiameter;
    private int gearRatio;
    private int threshold;
    private int sides;
    private double slowDownStart;
    private double fineTuneStart;
    private double powerMin;
    private double conversionFactor;
    private DcMotor[] motors;
    private int[] targets;

    void run(double target) {

        final int TICKS_PER_REVOLUTION   = 1120;
        final double K_FAST              = 1 / (slowDownStart * TICKS_PER_REVOLUTION);
        final double K_SLOW              = (1 / slowDownStart - powerMin / fineTuneStart) / TICKS_PER_REVOLUTION;
        final double TICK_OFFSET         = powerMin * slowDownStart * fineTuneStart * TICKS_PER_REVOLUTION / (fineTuneStart - powerMin * slowDownStart);

        switch (typePID) {
            case DRIVE:
                conversionFactor = TICKS_PER_REVOLUTION / (wheelDiameter * Math.PI * gearRatio);
                break;
            case TURN:
                break;
            case LIFT:
                break;
            default:
                break;
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
