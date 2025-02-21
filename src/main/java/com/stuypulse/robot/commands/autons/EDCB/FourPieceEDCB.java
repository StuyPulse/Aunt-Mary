package com.stuypulse.robot.commands.autons.EDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourPieceEDCB extends SequentialCommandGroup {
    
    public FourPieceEDCB(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on E
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine(),

            // To HP, Score on D
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new ElevatorToFeed(),
                new ArmToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new ScoreRoutine(),

            // To HP, Score on C
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new ElevatorToFeed(),
                new ArmToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
            new ScoreRoutine(),

            // To HP, Score on B
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
                new ElevatorToFeed(),
                new ArmToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[6]),
            new ScoreRoutine()

        );

    }

}
