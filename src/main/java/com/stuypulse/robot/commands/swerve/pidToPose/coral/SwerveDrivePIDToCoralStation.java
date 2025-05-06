/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.Arrays;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SwerveDrivePIDToCoralStation extends ParallelCommandGroup {
    private static Boolean isLeftSideOfStation = null;
    private static LEDPattern led = null;

    public SwerveDrivePIDToCoralStation(boolean isCD) {
        addCommands(
            new SwerveDrivePIDToPose(() -> Field.CoralStation.getCoralStation(isCD).getTargetPose())
        );
    }

    public SwerveDrivePIDToCoralStation(Gamepad driver) {
        addCommands(
            new SwerveDrivePIDToPose(() -> getCoralStationPoseWithDriverInput(driver)),
            new LEDApplyPattern(() -> led).onlyIf(() -> led != null)
        );

    }
        
    private static Pose2d getCoralStationPoseWithDriverInput(Gamepad driver) {
        Pose2d[] sides = new Pose2d[] {
            Field.CoralStation.getClosestCoralStation().getTargetPose(true),
            Field.CoralStation.getClosestCoralStation().getTargetPose(false)
        };

        if (driver.getLeftX() < -Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
            isLeftSideOfStation = true;
            led = Settings.LED.CORAL_STATION_ALIGN_COLOR_LEFT;
            return sides[0];
        } else if (driver.getLeftX() > Settings.Driver.CORAL_STATION_OVERRIDE_DEADBAND) {
            isLeftSideOfStation = false;
            led = Settings.LED.CORAL_STATION_ALIGN_COLOR_RIGHT;
            return sides[1];
        } else {
            return (CommandSwerveDrivetrain.getInstance().getPose().nearest(Arrays.asList(sides)));
        }
    }
}