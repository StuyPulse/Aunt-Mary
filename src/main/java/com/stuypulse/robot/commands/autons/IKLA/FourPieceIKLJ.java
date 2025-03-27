package com.stuypulse.robot.commands.autons.IKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignAuton;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FourPieceIKLJ extends SequentialCommandGroup {
    
    public FourPieceIKLJ(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on I
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.I, 4, true)
                    .withTranslationalConstraints(2.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON)
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.25),
            new ShooterStop(),

            // To HP, Score K
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_HP_COLOR)),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.K, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.125),
            new ShooterStop(),

            // To HP, Score L
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_HP_COLOR)),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop()))
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.L, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.125),
            new ShooterStop(),

           // To HP, Score J
           new ParallelCommandGroup(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_HP_COLOR)),
            new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(
                    new SuperStructureFeed()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
            ),
            new ParallelCommandGroup(
                new ShooterAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral() || Funnel.getInstance().hasCoral())
                    .andThen(
                        new ParallelCommandGroup(
                            new SwerveDriveCoralScoreAlignAuton(CoralBranch.J, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 5),
                            // new SwerveDrivePIDToBranchScore(CoralBranch.E, 4, true)
                            //     .withTranslationalConstraints(4, 5.5)
                            //     .withTimeout(5)
                            //     .deadlineFor(new LEDApplyPattern(CoralBranch.E.isLeftPegFieldRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                                    .andThen(
                                        new SuperStructureCoralL4Front())
                                            .andThen(new SuperStructureWaitUntilAtTarget())
                        )
                )
            ),

            new ShooterShootL4Front(),
            new WaitCommand(0.125),
            new ShooterStop(),

            new ParallelCommandGroup(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
                .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_HP_COLOR)),
            new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(
                    new SuperStructureFeed()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
            )

        );

    }

}