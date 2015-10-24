package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Andrew on 10/23/2015.
 */

// stores all the controller variables and methods
public class Controller {
    // controller constants
    private final float C_TOP_THRESHOLD = 0.85f;
    private final float C_BOTTOM_THRESHOLD = 0.05f;
    // converted controller stick values
    public float    C1_stickRx, // controller 1's converted right stick value in the x direction
                    C1_stickRy, // controller 1's converted right stick value in the y direction
                    C1_stickLx, // controller 1's converted left stick value in the x direction
                    C1_stickLy, // controller 1's converted left stick value in the y direction
                    C2_stickRx, // controller 2's converted right stick value in the x direction
                    C2_stickRy, // controller 2's converted right stick value in the y direction
                    C2_stickLx, // controller 2's converted left stick value in the x direction
                    C2_stickLy; // controller 2's converted left stick value in the y direction
    public float    C1_triggerR, // controller 1's right trigger value
                    C1_triggerL, // controller 1's left trigger value
                    C2_triggerR, // controller 2's right trigger value
                    C2_triggerL; // controller 2's left trigger value
    public double   C1_AngleR, // controller 1's right stick angle
                    C1_AngleL, // controller 1's left stick angle
                    C2_AngleR, // controller 2's right stick angle
                    C2_AngleL; // controller 2's left stick angle


    // converts all of the controller sticks into more sensitive values
    // use a negative value for y axis since controller reads -1 when pushed forward
    private double convertStick(double controllerValue) {
        double sigma = controllerValue * Math.PI / 2 / C_TOP_THRESHOLD;
        return Math.sin(sigma);
    }

    // updates the converted controller stick values as long as they are above the bottom threshold
    public void updateSticks(float C1_stickRx, float C1_stickRy, float C1_stickLx, float C1_stickLy, float C2_stickRx, float C2_stickRy, float C2_stickLx, float C2_stickLy) {
        this.C1_stickRx = (C1_stickRx > C_BOTTOM_THRESHOLD) ? (float)convertStick(C1_stickRx) : 0.0f;
        this.C1_stickRy = (C1_stickRy > C_BOTTOM_THRESHOLD) ? (float)convertStick(C1_stickRy) : 0.0f;
        this.C1_stickLx = (C1_stickLx > C_BOTTOM_THRESHOLD) ? (float)convertStick(C1_stickLx) : 0.0f;
        this.C1_stickLy = (C1_stickLy > C_BOTTOM_THRESHOLD) ? (float)convertStick(C1_stickLy) : 0.0f;
        this.C2_stickRx = (C2_stickRx > C_BOTTOM_THRESHOLD) ? (float)convertStick(C2_stickRx) : 0.0f;
        this.C2_stickRy = (C2_stickRy > C_BOTTOM_THRESHOLD) ? (float)convertStick(C2_stickRy) : 0.0f;
        this.C2_stickLx = (C2_stickLx > C_BOTTOM_THRESHOLD) ? (float)convertStick(C2_stickLx) : 0.0f;
        this.C2_stickLy = (C2_stickLy > C_BOTTOM_THRESHOLD) ? (float)convertStick(C2_stickLy) : 0.0f;
        C1_AngleR = Math.toDegrees(Math.atan(C1_stickRy/C1_stickRx));
        C1_AngleL = Math.toDegrees(Math.atan(C1_stickLy/C1_stickLx));
        C2_AngleR = Math.toDegrees(Math.atan(C2_stickRy/C2_stickRx));
        C2_AngleL = Math.toDegrees(Math.atan(C2_stickLy/C2_stickLx));
    }

    public void updateTriggers(float C1_triggerR, float C1_triggerL, float C2_triggerR, float C2_triggerL) {
        this.C1_triggerR = C1_triggerR;
        this.C1_triggerL = C1_triggerL;
        this.C2_triggerR = C2_triggerR;
        this.C2_triggerL = C2_triggerL;
    }

    public boolean isTriggerPressed(float trigger) {
        return (trigger > 0) ? true : false;
    }
}
