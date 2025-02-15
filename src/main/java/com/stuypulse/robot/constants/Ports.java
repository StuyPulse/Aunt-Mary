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
        int PIVOT = 7;
        int ROLLER = 8;
        int ABSOLUTE_ENCODER = 22;
    }

    public interface Climb {
        int MOTOR = 9;
        int ABSOLUTE_ENCODER = 0;
    }

    public interface Arm {
        int MOTOR = 12;
        int ABSOLUTE_ENCODER = 7;
    }

    public interface Shooter {
        int MOTOR = 13;
        int IR_SENSOR = 5;
    }

    public interface Funnel {
        int MOTOR = 19;
        int IR = 9;
    }

    public interface Elevator {
        int MOTOR = 10;
        int BOTTOM_SWITCH = 8;
    }

    public interface LED {
        int LED_PORT = 0;
    }
}