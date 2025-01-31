/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
        //should these be final?
    }

    public interface Froggy {
        int ROLLER_PORT = 0; // CHANGE
        int PIVOT_PORT = 0; // CHANGE
        int PIVOT_ENCODER = 0;
    }
}
