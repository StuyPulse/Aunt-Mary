/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;


import com.stuypulse.stuylib.network.SmartNumber;

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
            SmartNumber kP = new SmartNumber("Arm/PID/kP", 0.0);
            SmartNumber kI = new SmartNumber("Arm/PID/kI", 0);
            SmartNumber kD = new SmartNumber("Arm/PID/kD", 0);

        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Arm/PID/kP", 0.0);
            SmartNumber kV = new SmartNumber("Arm/PID/kI", 0);
            SmartNumber kA = new SmartNumber("Arm/PID/kD", 0);
            SmartNumber kG = new SmartNumber("Arm/PID/kG", 0);
        }

        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(0);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(0);

        Rotation2d FUNNEL_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d BARGE_ANGLE = Rotation2d.fromDegrees(0);
        double GEAR_RATIO = 0;
        double ARM_OFFSET = 0;
        double ENCODER_OFFSET = 0;
        SmartNumber PID_RAMPING = new SmartNumber("Arm/PID_RAMP",0);
        SmartNumber FF_RAMPING = new SmartNumber("Arm/FF_RAMP", 0);
        SmartNumber CURRENT_RAMPING = new SmartNumber("Arm/CURRENT_RAMP",0);


        public interface MotionMagic{
            double MAX_VEL = 0;
            double MAX_ACCEL = 0;
            double JERK = 0;
        }
    }
}
