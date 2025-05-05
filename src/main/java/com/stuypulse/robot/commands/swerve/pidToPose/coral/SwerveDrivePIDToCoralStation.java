
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.CoralStation;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDrivePIDToCoralStation extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToCoralStation(boolean isCD) {
        super(() -> Field.CoralStation.getCoralStation(isCD).getTargetPose());
    }

    public SwerveDrivePIDToCoralStation(Gamepad driver) {
        super(getCoralStationPoseWithDriverInput(driver));
    }
        
    private static Supplier<Pose2d> getCoralStationPoseWithDriverInput(Gamepad driver) {
        return () -> { 
            if (driver.getLeftX() < -Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
                return Field.CoralStation.getClosestCoralStation().getTargetPose(true);
            } else if (driver.getLeftX() > Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
                return Field.CoralStation.getClosestCoralStation().getTargetPose(false);
            } else {
                return Field.CoralStation.getClosestCoralStation().getTargetPose();
            }
        };
    }
}