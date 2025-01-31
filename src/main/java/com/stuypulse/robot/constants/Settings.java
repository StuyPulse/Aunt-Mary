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
    
    double DT = 0.020;

    public interface Elevator {
        SmartNumber MAX_VELOCITY_METERS_PER_SECOND = new SmartNumber("Elevator/Max Velocity (m per s)", 1.0);
        SmartNumber MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = new SmartNumber("Elevator/Max Accel (m per s^2)", 2.0);

        // CHANGE
        double HANDOFF_HEIGHT_METERS = 0.1;
        double L2_HEIGHT_METERS = 0.25;
        double L3_HEIGHT_METERS = 0.5;
        double L4_HEIGHT_METERS = 0.75;

        double FEED_HEIGHT_METERS = 0.4;

        double RAMP_RATE = 0.1;

        // FIND OUT REAL GEAR RATIO
        double GEAR_RATIO = 1.0/5.0;
        double CURRENT_LIMIT = 5.0;

        // Magic motion, change RPS
        double TARGET_CRUISE_VELOCITY = 0.0; //Rotations Per Second
        double TARGET_ACCELERATION = 0.0; //Rotations Per Second^2
        double TARGET_JERK = 0.0; //Rotations Per Second^3

        SmartNumber HEIGHT_TOLERANCE_METERS = new SmartNumber("Elevator/Height Tolerance (m)", 0.02);
    
        public interface PID {
            SmartNumber kP = new SmartNumber("Elevator/Controller/kP",10);
            SmartNumber kI = new SmartNumber("Elevator/Controller/kI",0.0);
            SmartNumber kD = new SmartNumber("Elevator/Controller/kD",0.2);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Elevator/Controller/kS",0.20506);
            SmartNumber kV = new SmartNumber("Elevator/Controller/kV",3.7672);
            SmartNumber kA = new SmartNumber("Elevator/Controller/kA", 0.27);
            SmartNumber kG = new SmartNumber("Elevator/Controller/kG", 1.37);
        }
        
        public interface Simulation {
            double SCALE_FACTOR = 0.5 + 2.5/77;
        }
    }
}
