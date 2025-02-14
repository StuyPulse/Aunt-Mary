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
        int ROLLER = 3;
        int PIVOT = 4;
        int ABSOLUTE_ENCODER = 5;
    }

    public interface Climb {
        int MOTOR = 6;
        int ABSOLUTE_ENCODER = 7;
    }

    public interface Arm {
        int MOTOR = 8;
        int ABSOLUTE_ENCODER = 9;
    }

    public interface Shooter {
        int MOTOR = 17;
        int IR_SENSOR = 11;
    }

    public interface Funnel {
        int MOTOR = 12;
        int IR = 13;
    }

    public interface Elevator {
        int MOTOR = 14;
        int BOTTOM_SWITCH = 15;
    }

    public interface LED {
        int LED_PORT = 9;
    }
}
