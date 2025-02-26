package com.stuypulse.robot.commands.autons.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ThreeHalfPieceJKL extends SequentialCommandGroup {
    
    public ThreeHalfPieceJKL(PathPlannerPath... paths) {

        addCommands(

            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on J
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Score on K
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new ElevatorToFeed().alongWith(new ArmToFeed())
            ),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Score on L
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new ElevatorToFeed().alongWith(new ArmToFeed())
            ),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // To HP, Acquire Coral
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
                new ElevatorToFeed().alongWith(new ArmToFeed())
            )

        );

    }

}
