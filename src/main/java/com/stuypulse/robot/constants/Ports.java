/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
        // should these be final?
    }

    public interface Froggy {
        int ROLLER_PORT = 3; // CHANGE
        int PIVOT_PORT = 4; // CHANGE
        int PIVOT_ENCODER = 5;
    }

    public interface Climb {
        int CLIMB_MOTOR = 6;
        int CLIMB_ENCODER = 7;
    }

    public interface Arm {
        int ARM_MOTOR = 8;
        int ARM_ENCODER = 9;
    }

    // Set values later
    public interface Shooter {
        int MOTOR = 17;
        int RECEIVER = 11;
    }

    // Set values later
    public interface Funnel {
        int MOTOR = 12;
        int IR = 13;
    }

    public interface Elevator {
        int MOTOR = 14;
        int BOTTOM_SWITCH = 15;
        int TOP_SWITCH = 16;
    }

    public interface LED {
        int LED_PORT = 10;
    }
}
