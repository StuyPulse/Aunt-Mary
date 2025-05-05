
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDrivePIDToCoralStation extends SwerveDrivePIDToPose {
    private static Boolean isLeft = null;

    public SwerveDrivePIDToCoralStation(boolean isCD) {
        super(() -> Field.CoralStation.getCoralStation(isCD).getTargetPose());
    }

    public SwerveDrivePIDToCoralStation(Gamepad driver) {
        super(() -> getCoralStationPoseWithDriverInput(driver));
    }
        
    private static Pose2d getCoralStationPoseWithDriverInput(Gamepad driver) {
        if (driver.getLeftX() < -Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
            isLeft = true;
            return Field.CoralStation.getClosestCoralStation().getTargetPose(true);
        } else if (driver.getLeftX() > Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
            isLeft = false;
            return Field.CoralStation.getClosestCoralStation().getTargetPose(false);
        } else {
            return (isLeft != null && isLeft ? 
                Field.CoralStation.getClosestCoralStation().getTargetPose(true) : 
                Field.CoralStation.getClosestCoralStation().getTargetPose(false));
        }
    }
}