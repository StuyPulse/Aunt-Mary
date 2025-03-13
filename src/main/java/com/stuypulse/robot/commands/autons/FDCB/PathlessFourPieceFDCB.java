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
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PathlessFourPieceFDCB extends SequentialCommandGroup {
    
    public PathlessFourPieceFDCB(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on F
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.F, 4, true)
                    .withTranslationalConstraints(2.35, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.F.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

            // To HP, Score D
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.D, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

            // To HP, Score C
            new ParallelCommandGroup(
                new SwerveDrivePIDToCoralStation(true),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop()))
            ),
            new ParallelCommandGroup(
                new SwerveDriveCoralScoreAlignAuton(CoralBranch.C, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT, 3),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

           // To HP, Score B
           new ParallelCommandGroup(
            new SwerveDrivePIDToCoralStation(true),
            new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(
                    new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                )
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral()),
                new ShooterSetAcquireCoral()
                    .andThen(
                        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                            .andThen(new ShooterStop()))
            ),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.B, 4, true)
                    .withTranslationalConstraints(5.85, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(4)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.B.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                    .andThen(
                        new SuperStructureCoralL4Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])

        );

    }

}
