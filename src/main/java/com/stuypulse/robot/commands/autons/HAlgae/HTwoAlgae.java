
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.autons.HAlgae;

import com.stuypulse.robot.commands.ReefAlgaePickupRoutineFront;
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
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveWithRobotRelativeSpeeds;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToCatapult;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class HTwoAlgae extends SequentialCommandGroup {
    
    public HTwoAlgae(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on H
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.H, 4, true)
                    .withTranslationalConstraints(2, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION)
                    .withTimeout(3)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ReefAlgaePickupRoutineFront()
                .withTimeout(2.5)
                .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)),
            new ShooterHoldAlgae(),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureCatapultReady())
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SuperStructureCatapultShoot()
                                .andThen(new SuperStructureWaitUntilCanCatapult()
                                        .andThen(new ShooterShootAlgae()))),
                new SwerveDrivePIDToCatapult(Settings.Swerve.Alignment.Targets.Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_LONG)
            ),

            new SuperStructureCatapultShoot()
                .andThen(new SuperStructureWaitUntilCanCatapult())
                    .andThen(new ShooterShootAlgae()),

            new WaitCommand(0.1),

            // Acquire IJ Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new SuperStructureAlgaeL3Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ReefAlgaePickupRoutineFront()
                .withTimeout(2)
                .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)),
            new ShooterHoldAlgae(),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureCatapultReady())
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new WaitCommand(1))
                                .andThen(new SuperStructureCatapultShoot())
                                    .andThen(new SuperStructureWaitUntilCanCatapult())
                                        .andThen(new ShooterShootAlgae()),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                    .andThen(new SwerveDrivePIDToCatapult(Settings.Swerve.Alignment.Targets.Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_SHORT))
            ),

            new SuperStructureCatapultShoot()
                .andThen(new SuperStructureWaitUntilCanCatapult())
                    .andThen(new ShooterShootAlgae()),

            new WaitCommand(0.1),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
                .alongWith(new SuperStructureFeed().alongWith(new ShooterStop()))
                    .andThen(new SuperStructureWaitUntilAtTarget())

        );

    }

}
