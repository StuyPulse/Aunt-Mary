package com.stuypulse.robot.commands.autons.GAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBargeAllianceSide;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToBarge;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
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

public class OneGThreeAlgae extends SequentialCommandGroup {
    
    public OneGThreeAlgae(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on G
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.G, 4, true)
                    .withTranslationalConstraints(1.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.G.isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.15),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureAlgaeL2()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge(),
                new SuperStructureCatapultReady()
                    .andThen(new SuperStructureWaitUntilAtTarget()
                        .alongWith(new SwerveDriveWaitUntilAlignedToBargeAllianceSide()))
                    .andThen(new SuperStructureCatapultShoot()
                        .andThen(new SuperStructureWaitUntilCanCatapult()
                            .andThen(new ShooterShootAlgae())))
            ),
            new ShooterShootAlgae(),

            // Acquire IJ Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureAlgaeL3()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge(),
                new SuperStructureCatapultReady()
                    .andThen(new SuperStructureWaitUntilAtTarget()
                        .alongWith(new SwerveDriveWaitUntilAlignedToBargeAllianceSide()))
                    .andThen(new SuperStructureCatapultShoot()
                        .andThen(new SuperStructureWaitUntilCanCatapult()
                            .andThen(new ShooterShootAlgae())))
            ),

            // Acquire EF Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureAlgaeL3()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge(),
                new SuperStructureCatapultReady()
                    .andThen(new SuperStructureWaitUntilAtTarget()
                        .alongWith(new SwerveDriveWaitUntilAlignedToBargeAllianceSide()))
                    .andThen(new SuperStructureCatapultShoot()
                        .andThen(new SuperStructureWaitUntilCanCatapult()
                            .andThen(new ShooterShootAlgae())))
            )
        );

    }

}
