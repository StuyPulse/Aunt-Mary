package com.stuypulse.robot.commands.autons.GAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureBarge118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeBackwards;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeBackwardsAuton;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge118AllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapultAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilClearFromBarge118;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ClearAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ScoreAllianceSide;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToBarge;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;

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
                    .deadlineFor(new LEDApplyPattern(CoralBranch.G.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureAlgaeL2Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(true).withTimeout(2),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureBarge118()),
                new SwerveDrivePIDToBarge()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SwerveDriveNudgeBackwardsAuton().withTimeout(0.5))
                                .andThen(new ShooterShootAlgae())
            ),

            new WaitCommand(0.5),

            // Acquire IJ Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureAlgaeL3Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(true).withTimeout(2),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureBarge118()),
                new SwerveDrivePIDToBarge()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SwerveDriveNudgeBackwardsAuton().withTimeout(0.5))
                                .andThen(new ShooterShootAlgae())
            ),
            
            new WaitCommand(0.5),

            // Acquire EF Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureAlgaeL3Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(true).withTimeout(2),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureBarge118()),
                new SwerveDrivePIDToBarge()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SwerveDriveNudgeBackwardsAuton().withTimeout(0.5))
                                .andThen(new ShooterShootAlgae())
            ),

            new WaitCommand(0.5),

            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureAlgaeL3Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            )

        );

    }

}
