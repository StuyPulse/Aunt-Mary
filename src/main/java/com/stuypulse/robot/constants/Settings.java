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
    
    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d GOLF_TEE_ALGAE_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d L1_SCORING_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d PROCESSOR_SCORE_ANGLE = Rotation2d.fromDegrees(0);
        
        double ALGAE_INTAKE_SPEED = 0.0;
        double ALGAE_OUTTAKE_SPEED = 0.0;
        double CORAL_INTAKE_SPEED = 0.0;
        double CORAL_OUTTAKE_SPEED = 0.0;

        double CORAL_CURRENT_THRESHOLD = 0.0;
        double ALGAE_CURRENT_THRESHOLD = 0.0;

        double GEAR_RATIO = 0.0;
        double PIVOT_CURRENT_LIMIT = 0.0;
        double STATOR_CURRENT_LIMIT = 0.0;
        double ROLLER_CURRENT_THRESHOLD = 0.0;
        double ANGLE_OFFSET = 0.0;

        double ROLLER_FF_RAMPING = 0.0;
        double PIVOT_FF_RAMPING = 0.0;
	    double ROLLER_PID_RAMPING = 0.0;
	    double PIVOT_PID_RAMPING = 0.0;
		double ROLLER_DEBOUNCE_TIME = 0.0;
        
        double MINIMUM_ANGLE = 0.0;
        double MAXIMUM_ANGLE = 0.0;
		double MAGNET_OFFSET = 0.0;
		double ENCODER_GEAR_RATIO = 0.0;
        
        public interface PID{
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double kG = 0.0;
        }
        public interface FF{
            double kA = 0.0;
            double kS = 0.0;
            double kV = 0.0;
            double kG = 0.0;
        }
        public interface MotionMagic{
            double MAX_VELOCITY = 0.0;
            double MAX_ACCELERATION = 0.0;
            double MAX_JERK = 0.0;
        }
    }
}
