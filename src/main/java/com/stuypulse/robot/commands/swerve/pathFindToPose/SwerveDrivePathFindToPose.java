
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pathFindToPose;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import java.util.function.Supplier;

public class SwerveDrivePathFindToPose extends Command{
    private Supplier<Pose2d> targetPose;
    private Pose2d initialTargetPose;
    private Command pathFindCommand;

    public  SwerveDrivePathFindToPose(Supplier<Pose2d> targetPose) {
        this.targetPose = targetPose;
    }

    public SwerveDrivePathFindToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public static SwerveDrivePathFindToPose pathFindToNearestCoralStation() {
        return new SwerveDrivePathFindToPose(() -> Field.CoralStation.getClosestCoralStation().getTargetPose());
    }

    @Override
    public void initialize() {
        initialTargetPose = targetPose.get();
        pathFindCommand = AutoBuilder.pathfindToPose(initialTargetPose, Settings.Swerve.Constraints.DEFAULT_CONSTRAINTS);
        pathFindCommand.initialize();
    }

    @Override
    public void execute() {
        if (initialTargetPose.getTranslation().minus(targetPose.get().getTranslation()).getDistance(Translation2d.kZero) > 0.01
            || initialTargetPose.getRotation().minus(targetPose.get().getRotation()).getDegrees() > 1) {
            initialTargetPose = targetPose.get();
            pathFindCommand = AutoBuilder.pathfindToPose(initialTargetPose, Settings.Swerve.Constraints.DEFAULT_CONSTRAINTS);
            pathFindCommand.initialize();
        }
        else {
            pathFindCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return pathFindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathFindCommand.end(interrupted);
    }
}
