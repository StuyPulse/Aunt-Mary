package com.stuypulse.robot.commands.swerve.pathFindToPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePathFindToPose extends Command{
    public static Command pathFindTo(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, Settings.Swerve.Constraints.DEFAULT_CONSTRAINTS);
    }
}
