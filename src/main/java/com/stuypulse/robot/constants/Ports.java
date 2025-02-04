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

    public interface Climb {
        int CLIMB_MOTOR = 0;
        int CLIMB_ENCODER = 0;
    }

    public interface Arm {
        int ARM_MOTOR = 0;
        int ARM_ENCODER = 0;
    }
  
    // Set values later
    public interface Shooter {
        int MOTOR = 0;
        int RECEIVER = 1;
    }
    
    // Set values later
    public interface Funnel {
        int MOTOR = 0;
        int IR = 1;
    }
      
    public interface Elevator {
        int MOTOR = 0;
        int BOTTOM_SWITCH = 1;
        int TOP_SWITCH = 2;
    }
}

