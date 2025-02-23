package com.stuypulse.robot.commands.autons.HAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL2;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL3;
import com.stuypulse.robot.commands.arm.algae.ArmToBarge;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL2;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL3;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneHThreeAlgae extends SequentialCommandGroup {
    
    public OneHThreeAlgae(PathPlannerPath... paths) {

        addCommands(

            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on H
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new ElevatorToAlgaeL2().alongWith(new ArmToAlgaeL2())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterAcquireAlgae(),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new ElevatorToBarge().alongWith(new ArmToBarge())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterShootAlgae(),
            
            // Acquire IJ Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterAcquireAlgae(),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new ElevatorToBarge().alongWith(new ArmToBarge())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterShootAlgae(),

            // Acquire EF Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
                new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterAcquireAlgae(),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[6]),
                new ElevatorToBarge().alongWith(new ArmToBarge())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterShootAlgae()

        );

    }

}
