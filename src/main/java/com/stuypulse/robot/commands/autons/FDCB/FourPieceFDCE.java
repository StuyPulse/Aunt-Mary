package com.stuypulse.robot.commands.autons.FDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignAuton;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutineFront;
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

public class FourPieceFDCE extends SequentialCommandGroup {
    
    public FourPieceFDCE(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on F
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.F, 4, true)
                    .withTranslationalConstraints(2.5, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION)
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

            // To HP, Score D
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
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.D, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 2.5),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

            // To HP, Score C
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
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.C, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 2.5),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

           // To HP, Score E
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
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new ParallelCommandGroup(
                            new SwerveDriveCoralScoreAlignWithClearance(() -> CoralBranch.E, 4, true, SuperStructureState.L4_FRONT),
                                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                                    .andThen(
                                        new SuperStructureCoralL4Front())
                                            .andThen(new SuperStructureWaitUntilAtTarget())
                        )
                )
            ),

            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])


        );

    }

}