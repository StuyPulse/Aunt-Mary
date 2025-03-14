package com.stuypulse.robot.commands.autons.FDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterSetAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignAuton;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToCoralStation;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FunnelFourPiece extends SequentialCommandGroup {
    
    public FunnelFourPiece(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on F
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.F, 4, true)
                    .withTranslationalConstraints(3, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.F.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),

            // To HP, Score D
            new ParallelCommandGroup(
                new ShooterShootL4Front()
                    .andThen(new WaitCommand(0.125))
                        .andThen(new ShooterStop()),
                new WaitCommand(0.1)
                    .andThen(
                        CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                    ),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
            new WaitUntilCommand(() -> Funnel.getInstance().hasCoral())
                .raceWith(
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()))
                .andThen(
                    new WaitUntilCommand(() -> !Funnel.getInstance().hasCoral())
                        .andThen(
                            new SwerveDrivePIDToBranchScore(CoralBranch.D, 4, true)
                                .withTolerance(Units.inchesToMeters(3.0), Units.inchesToMeters(3.0), Units.degreesToRadians(5.0))
                                .withTranslationalConstraints(6, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                                .withTimeout(3)
                                .deadlineFor(new LEDApplyPattern(CoralBranch.D.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
                        )
                    ),
            // new ParallelCommandGroup(
            //     new WaitUntilCommand(() -> Funnel.getInstance().hasCoral())
            //         .raceWith(
            //             new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
            //         )
            //         .andThen(new SwerveDriveCoralScoreAlignAuton(
            //             CoralBranch.D, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3)
            //             ),
                new ShooterSetAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())
                                .andThen(
                                    new SuperStructureCoralL4Front()
                                        .andThen(new SuperStructureWaitUntilAtTarget())
                                )
                    )
            ),

            // To HP, Score C
            new ParallelCommandGroup(
                new ShooterShootL4Front()
                    .andThen(new WaitCommand(0.125))
                        .andThen(new ShooterStop()),
                new WaitCommand(0.1)
                    .andThen(
                        new SwerveDrivePIDToCoralStation(true)
                    ),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
            new WaitUntilCommand(() -> Funnel.getInstance().hasCoral())
                .raceWith(
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()))
                .andThen(
                    new WaitUntilCommand(() -> !Funnel.getInstance().hasCoral())
                        .andThen(
                            new SwerveDrivePIDToBranchScore(CoralBranch.C, 4, true)
                                .withTolerance(Units.inchesToMeters(3.0), Units.inchesToMeters(3.0), Units.degreesToRadians(5.0))
                                .withTranslationalConstraints(6, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                                .withTimeout(3)
                                .deadlineFor(new LEDApplyPattern(CoralBranch.C.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
                        )
                    ),
            // new ParallelCommandGroup(
            //     new WaitUntilCommand(() -> Funnel.getInstance().hasCoral())
            //         .raceWith(
            //             new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
            //         )
            //         .andThen(new SwerveDriveCoralScoreAlignAuton(
            //             CoralBranch.C, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3)
            //             ),
                new ShooterSetAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())
                                .andThen(
                                    new SuperStructureCoralL4Front()
                                        .andThen(new SuperStructureWaitUntilAtTarget())
                                )
                    )
            ),

           // To HP, Score B
           new ParallelCommandGroup(
            new ShooterShootL4Front()
                .andThen(new WaitCommand(0.125))
                    .andThen(new ShooterStop()),
            new WaitCommand(0.1)
                .andThen(
                    new SwerveDrivePIDToCoralStation(true)
                ),
            new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(
                    new SuperStructureFeed()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
        ),
        new ParallelCommandGroup(
            new WaitUntilCommand(() -> Funnel.getInstance().hasCoral())
                .raceWith(
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()))
                .andThen(
                    new WaitUntilCommand(() -> !Funnel.getInstance().hasCoral())
                        .andThen(
                            new SwerveDrivePIDToBranchScore(CoralBranch.B, 4, true)
                                .withTolerance(Units.inchesToMeters(3.0), Units.inchesToMeters(3.0), Units.degreesToRadians(5.0))
                                .withTranslationalConstraints(6, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                                .withTimeout(3)
                                .deadlineFor(new LEDApplyPattern(CoralBranch.B.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
                        )
                    ),
            new ShooterSetAcquireCoral()
                .andThen(
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                        .andThen(new ShooterStop())
                            .andThen(
                                new SuperStructureCoralL4Front()
                                    .andThen(new SuperStructureWaitUntilAtTarget())
                            )
                )
        ),

            new ParallelCommandGroup(
                new ShooterShootL4Front()
                    .andThen(new WaitCommand(0.125))
                        .andThen(new ShooterStop()),
                new WaitCommand(0.1)
                    .andThen(
                        CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
                    ),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            )

        );

    }

}
