/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    
    public interface Algae {
        double STOW_ANGLE = 0.0;
        double ALGAE_GROUND_PICKUP_ANGLE = 0.0;
        double CORAL_GROUND_PICKUP_ANGLE = 0.0;
        double GOLF_TEE_ALGAE_PICKUP_ANGLE = 0.0;
        double L1_SCORING_ANGLE = 0.0;
        double PROCESSOR_SCORE_ANGLE = 0.0;
        double ALGAE_INTAKE_SPEED = 0.0;
        double ALGAE_OUTTAKE_SPEED = 0.0;
        double CORAL_INTAKE_SPEED = ALGAE_OUTTAKE_SPEED; // these variables are in order to avoid confusion with the coral and algae intake and outtake speeds
        double CORAL_OUTTAKE_SPEED = ALGAE_INTAKE_SPEED;

        Pose2d positionOffset = new Pose2d(); // idk what this is supposed to be
        double PIVOT_CURRENT_THRESHOLD = 0.0;

        double GEAR_RATIO = 0;
        double PIVOT_CURRENT_LIMIT = 0;
        double STATOR_CURRENT_LIMIT = 0;
        double ROLLER_CURRENT_THRESHOLD = 0;
        double ANGLE_OFFSET = 0;
        
        public interface PID{
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double MAX_VELOCITY = 0.0;
            double MAX_ACCELERATION = 0.0;
        }
        public interface FF{
            double kA = 0.0;
            double kS = 0.0;
            double kV = 0.0;
            double kG = 0.0;
        }
    }
}
