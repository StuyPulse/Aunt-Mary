/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Climb {
        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

        int CURRENT_LIMIT = 0;
		double GEAR_RATIO = 25;
        double RAMP_RATE = 0;

        double STOW_ANGLE = 0.0;
        double INTAKE_ANGLE = 0.0;
        double ACQUIRED_ANGLE = 0.0;
        double CLIMBED_ANGLE = 0.0;
        double CLIMB_ANGLE_TOLERANCE = 2.0;
    }
}
