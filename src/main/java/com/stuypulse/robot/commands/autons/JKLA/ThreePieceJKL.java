package com.stuypulse.robot.commands.autons.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.commands.superstructure.SuperStructureToFeed;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreePieceJKL extends SequentialCommandGroup {
    
    public ThreePieceJKL(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on J
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine(),

            // To HP, Score on K
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new SuperStructureToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new ScoreRoutine(),

            // To HP, Score on L
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new SuperStructureToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
            new ScoreRoutine()

        );

    }

}
