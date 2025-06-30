
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.autons.GAlgae;

import com.stuypulse.robot.commands.ReefAlgaePickupRoutineFront;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.scoring.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.scoring.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeSafe118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureBarge118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveWithRobotRelativeSpeeds;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBarge118Score;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToBarge118Auto;
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

public class GTwoAlgae extends SequentialCommandGroup {
    
    public GTwoAlgae(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on G
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.G, 4, true)
                    .withTranslationalConstraints(2, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION)
                    .withTimeout(1.5)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(Settings.Shooter.CORAL_SHOOT_TIME_AUTON),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ReefAlgaePickupRoutineFront()
                .withTimeout(2)
                .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge118Auto(Settings.Swerve.Alignment.Targets.Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_SHORT)
                    .withTranslationalConstraints(5, 8),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                            new SuperStructureBarge118()
                        )
                    ),

            new ShooterShootAlgae(),
            new WaitCommand(0.2),
            new SuperStructureAlgaeSafe118(),
            new WaitCommand(0.2),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
                .alongWith(new SuperStructureFeed()),

            // Acquire EF Algae, Score on Barge
            new ReefAlgaePickupRoutineFront()
                .withTimeout(1.5)
                .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge118Auto(Settings.Swerve.Alignment.Targets.Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_LONG)
                    .withTranslationalConstraints(4, 6),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                            new SuperStructureBarge118()
                        )
                    ),

                    new ShooterShootAlgae(),
                    new WaitCommand(0.2),
                    new SuperStructureAlgaeSafe118(),
                    new WaitCommand(0.2),
                    
                    new ParallelCommandGroup(
                        new SuperStructureFeed(),
                        CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
                    )

        );

    }

}
