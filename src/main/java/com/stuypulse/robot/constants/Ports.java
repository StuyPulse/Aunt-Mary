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
    }

    public interface Froggy {
        int PIVOT = 22;
        int ROLLER = 21;
        int ABSOLUTE_ENCODER = 0;
    }

    public interface Climb {
        int MOTOR = 20;
        // int ABSOLUTE_ENCODER = 0;
    }

    public interface Arm {
        int MOTOR = 32;
        int ABSOLUTE_ENCODER = 7;
    }

    public interface Shooter {
        int MOTOR = 42;
        int IR_SENSOR = 5;
    }

    public interface Funnel {
        int MOTOR = 61;
        int IR = 9;
    }

    public interface Elevator {
        int MOTOR = 31;
        int BOTTOM_SWITCH = 8;
    }

    public interface LED {
        int LED_PORT = 0;
    }
}