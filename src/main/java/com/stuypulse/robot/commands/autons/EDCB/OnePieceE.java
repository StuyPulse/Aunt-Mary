package com.stuypulse.robot.commands.autons.EDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.commands.superstructure.SuperStructureToFeed;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceE extends SequentialCommandGroup {
    
    public OnePieceE(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on E
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine()
            
        );

    }

}
