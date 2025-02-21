package com.stuypulse.robot.commands.autons.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superstructure.SuperStructureToFeed;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL4Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FourPieceJKLA extends SequentialCommandGroup {
    
    public FourPieceJKLA(PathPlannerPath... paths) {

        addCommands(

            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on J
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new SuperStructureToL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDrivePIDToNearestBranch(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Score on K
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new SuperStructureToFeed()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new SuperStructureToL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDrivePIDToNearestBranch(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Score on L
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new SuperStructureToFeed()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new SuperStructureToL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDrivePIDToNearestBranch(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Score on A
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
                new SuperStructureToFeed()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[6]),
                new SuperStructureToL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDrivePIDToNearestBranch(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop()

        );

    }

}
