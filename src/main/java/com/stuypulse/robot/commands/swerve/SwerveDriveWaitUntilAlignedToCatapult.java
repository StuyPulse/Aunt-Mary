/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.math.Angle;
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
    private final CommandSwerveDrivetrain swerve;
    private final BStream isAligned;

    public SwerveDriveWaitUntilAlignedToCatapult() {
        swerve = CommandSwerveDrivetrain.getInstance();
        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
    }

    private double getTargetX() {
        return swerve.getPose().getX() < Field.LENGTH / 2
            ? Field.LENGTH / 2 - Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT
            : Field.LENGTH / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT;
    }

    private Rotation2d getTargetAngle() {
        return swerve.getPose().getX() < Field.LENGTH /2
            ? Rotation2d.k180deg.plus(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_CATAPULT)
            : Rotation2d.kZero.minus(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_CATAPULT);
    }

    private boolean isAligned() {
        Pose2d pose = swerve.getPose();
        return Math.abs(getTargetX() - pose.getX()) < Alignment.Tolerances.X_TOLERANCE_BARGE
            && Math.abs(pose.getRotation().minus(getTargetAngle()).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_BARGE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }
}
