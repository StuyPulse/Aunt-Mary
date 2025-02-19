package com.stuypulse.robot.commands.autons.misc;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceH extends SequentialCommandGroup {
    
    public OnePieceH(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on H
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine()
            
        );

    }

}
