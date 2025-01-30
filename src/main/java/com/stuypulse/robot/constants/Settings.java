/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {  
    public interface Shooter {
        double MAX_SHOOTER_RPM = 6000; // Max RPM of KrakenX60 (rpm)
        double TARGET_SHOOTER_RPM = 6000; // Target RPM of KrakenX60 (rpm)
        double BB_DEBOUNCE = 0.0; 
        
        public interface PID {
            // ADJUST LATER
            SmartNumber kP = new SmartNumber("kP", 0);
            SmartNumber kI = new SmartNumber("kI", 0);
            SmartNumber kD = new SmartNumber("kD", 0);
        }

        public interface FeedForward {
            // ADJUST LATER
            SmartNumber kS = new SmartNumber("kS", 0);
            SmartNumber kV = new SmartNumber("kV", 0);
            SmartNumber kA = new SmartNumber("kA", 0);
        }

        double GEAR_RATIO = 0.0;
        double DRIVE_CURRENT_THRESHOLD = 80;
        double DRIVE_CURRENT_LIMIT = 120;
    }

    public interface Funnel {
        double MAX_FUNNEL_RPM = 6000; // Max RPM of KrakenX60 (rpm)
        double TARGET_FUNNEL_RPM = 6000; // Target RPM of KrakenX60 (rpm)
        double BB_DEBOUNCE = 0.0;

        public interface PID {
            // ADJUST LATER
            SmartNumber kP = new SmartNumber("kP", 0);
            SmartNumber kI = new SmartNumber("kI", 0);
            SmartNumber kD = new SmartNumber("kD", 0);
        }

        public interface FeedForward {
            // ADJUST LATER
            SmartNumber kS = new SmartNumber("kS", 0);
            SmartNumber kV = new SmartNumber("kV", 0);
            SmartNumber kA = new SmartNumber("kA", 0);
        }
    
        double GEAR_RATIO = 0.0;
        double DRIVE_CURRENT_THRESHOLD = 80;
        double DRIVE_CURRENT_LIMIT = 120;
    }
}
