package com.stuypulse.robot.commands.autons.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ThreePieceJKL extends SequentialCommandGroup {
    
    public ThreePieceJKL(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on J
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.I, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.5),
            new ShooterStop(),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitCommand(1.5)
                    .andThen(
                        new ElevatorToFeed().alongWith(new ArmToFeed())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),

            new WaitCommand(0.25),

            // To HP, Score on K
            new ParallelCommandGroup(
                new ShooterSetAcquire().until(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new WaitCommand(0.1),
                        new ShooterStop()), // change ts
                new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.K, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.5),
            new ShooterStop(),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(1.5)
                    .andThen(
                        new ElevatorToFeed().alongWith(new ArmToFeed())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new WaitCommand(0.25),

            // To HP, Score on L
            new ParallelCommandGroup(
                new ShooterSetAcquire().until(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new WaitCommand(0.1),
                        new ShooterStop()),
                new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.L, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.5),
            new ShooterStop(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])

        );

    }

}
