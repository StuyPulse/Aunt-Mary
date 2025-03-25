
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.autons.GAlgae;

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
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveWithRobotRelativeSpeeds;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToBarge;
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

public class OneGTwoAlgae extends SequentialCommandGroup {
    
    public OneGTwoAlgae(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on G
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.G, 4, true)
                    .withTranslationalConstraints(1.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON)
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
                    .andThen(new SuperStructureCatapultReady()),
                new SwerveDrivePIDToBarge()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SwerveDriveDriveWithRobotRelativeSpeeds(-Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND_AUTON, 0, 0).withTimeout(0.5))
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
                    .andThen(new SuperStructureCatapultReady()),
                new SwerveDrivePIDToBarge()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                            .andThen(new SwerveDriveDriveWithRobotRelativeSpeeds(-Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND_AUTON, 0, 0).withTimeout(0.5))
                                .andThen(new ShooterShootAlgae())
            ),
            
            new WaitCommand(0.5),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new ParallelCommandGroup(
                new WaitCommand(0.2)
                    .andThen(
                        new SuperStructureFeed()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            )

        );

    }

}
