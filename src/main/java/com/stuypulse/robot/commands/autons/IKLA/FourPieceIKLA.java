package com.stuypulse.robot.commands.autons.IKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearanceAuton;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FourPieceIKLA extends SequentialCommandGroup {
    
    public FourPieceIKLA(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on I
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.I, 4, true)
                    .withTranslationalConstraints(1.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(2)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.I.isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.2),
            new ShooterStop(),

            // To HP, Score K
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitUntilCommand(() -> CommandSwerveDrivetrain.getInstance().isClearFromReef())
                    .andThen(
                        new ElevatorToFeed().alongWith(new ArmToFeed())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquire()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignWithClearanceAuton(CoralBranch.K, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 5),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.2),
            new ShooterStop(),

            // To HP, Score L
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitUntilCommand(() -> CommandSwerveDrivetrain.getInstance().isClearFromReef())
                    .andThen(
                        new ElevatorToFeed().alongWith(new ArmToFeed())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquire()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop()))
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignWithClearanceAuton(CoralBranch.L, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 5),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.2),
            new ShooterStop(),

           // To HP, Score A
           new ParallelCommandGroup(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new WaitUntilCommand(() -> CommandSwerveDrivetrain.getInstance().isClearFromReef())
                .andThen(
                    new ElevatorToFeed().alongWith(new ArmToFeed())
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquire()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop()))
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.A, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ElevatorToL4Front().alongWith(new ArmToL4Front())
                            .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                    )
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.2),
            new ShooterStop(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])

        );

    }

}
