package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Andrew on 9/13/2015.
 */
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // starts program when start button is pushed
        waitForStart();
        // sleep waits until a time has passed

        // runs while stop button isn't pushed
        while(opModeIsActive()) {

        }
        // difference between setPowerFLoat and setting power to 0?
    }
}
