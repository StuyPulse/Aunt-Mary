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

    public interface Arm {

            
        public interface PID {
            double kP = 0;
            double kI = 0;
            double kD = 0;

        }

        public interface FF {
            double kS = 0;
            double kV = 0;
            double kA = 0;
            double kG = 0;
        }

        double L2_ANGLE = 0;
        double L3_ANGLE = 0;
        double L4_ANGLE = 0;
        double FUNNEL_ANGLE = 0;
        double GEAR_RATIO = 0;

        public interface MotionMagic{
            double MAX_VEL = 0;
            double MAX_ACCEL = 0;
        }
    }
}
