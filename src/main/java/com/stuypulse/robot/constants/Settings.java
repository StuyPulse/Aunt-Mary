/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;


import edu.wpi.first.math.geometry.Rotation2d;

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

        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(0);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(0);

        Rotation2d FUNNEL_ANGLE = Rotation2d.fromDegrees(0);
        double GEAR_RATIO = 0;
        Rotation2d BARGE_ANGLE = Rotation2d.fromDegrees(0);
        double OFFSET = 0;
        double PID_RAMPING = 0;
        double FF_RAMPING = 0;

        public interface MotionMagic{
            double MAX_VEL = 0;
            double MAX_ACCEL = 0;
        }
    }
}
