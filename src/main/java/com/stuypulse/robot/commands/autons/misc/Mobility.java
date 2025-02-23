package com.stuypulse.robot.commands.autons.misc;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {
    
    public Mobility(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Drives straight
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])

        );

    }

}
