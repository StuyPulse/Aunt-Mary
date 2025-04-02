/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWaitUntilAlignedToCatapult extends Command{
    private final BStream isAligned;

    public SwerveDriveWaitUntilAlignedToCatapult() {
        isAligned = BStream.create(() -> isAlignedAllianceSide() || isAlignedOppositeAllianceSide())
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
    }

    private boolean isAlignedAllianceSide() {
        Pose2d pose = CommandSwerveDrivetrain.getInstance().getPose();
        return Math.abs((Field.LENGTH / 2 - Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT) - pose.getX())
            < Alignment.Tolerances.X_TOLERANCE_BARGE
            && Math.abs(pose.getRotation().minus(Rotation2d.k180deg).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_BARGE.getDegrees();
    }

    private boolean isAlignedOppositeAllianceSide() {
        Pose2d pose = CommandSwerveDrivetrain.getInstance().getPose();
        return Math.abs((Field.LENGTH / 2 + Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT) - pose.getX())
            < Alignment.Tolerances.X_TOLERANCE_BARGE
            && Math.abs(pose.getRotation().minus(Rotation2d.kZero).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_BARGE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }
}
