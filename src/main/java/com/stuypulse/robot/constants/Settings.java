/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {  
    public interface Shooter {
        SmartNumber ALGAE_FRONT_SPEED = new SmartNumber("Algae Target Speed", 1);
        SmartNumber CORAL_FRONT_SPEED = new SmartNumber("Coral Target Speed",0.75);
        SmartNumber ALGAE_BACK_SPEED = new SmartNumber("Algae Target Speed", 1);
        SmartNumber CORAL_BACK_SPEED = new SmartNumber("Coral Target Speed",0.75);
        SmartNumber ALGAE_ACQUIRE_SPEED = new SmartNumber("Algae Acquire Speed", 0.45);
        SmartNumber ALGAE_DEACQUIRE_SPEED = new SmartNumber("Algae Deacquire Speed", 0.45);
        SmartNumber CORAL_ACQUIRE_SPEED = new SmartNumber("Coral Acquire Speed", 0.3);

        double BB_DEBOUNCE = 0.0; 
        double CORAL_STALLING_DEBOUNCE = 0.0;
        double ALGAE_DEBOUNCE = 0.0;
        
        double RAMP_RATE = 0.0;

        double DRIVE_CURRENT_THRESHOLD = 30;
        double DRIVE_CURRENT_LIMIT = 40;
    }

    public interface Funnel {
        SmartNumber MOTOR_SPEED = new SmartNumber("Funnel Speed", 0.0);
        double BB_DEBOUNCE = 0.0;
        double FUNNEL_STALLING = 0.0;

        double RAMP_RATE = 0.0; 
    
        double GEAR_RATIO = 0.0;
        double DRIVE_CURRENT_THRESHOLD = 30;
        double DRIVE_CURRENT_LIMIT = 40;
    }
}
