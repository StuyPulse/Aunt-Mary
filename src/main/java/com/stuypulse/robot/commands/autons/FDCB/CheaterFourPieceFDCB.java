
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.autons.FDCB;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterSetAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL2Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.pathFindToPose.SwerveDrivePathFindToPose;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToCoralStation;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class CheaterFourPieceFDCB extends SequentialCommandGroup {

public CheaterFourPieceFDCB(PathPlannerPath... paths) {

    addCommands(

        // Score Preload on F
        new ParallelCommandGroup(
            new SwerveDrivePIDToBranchScore(CoralBranch.F, 4, true)
                .withTranslationalConstraints(3, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON)
                .withTimeout(1.75)
                .deadlineFor(new LEDApplyPattern(CoralBranch.F.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
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
                    SwerveDrivePathFindToPose.pathFindToNearestCoralStation()
                ),
            new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(
                    new SuperStructureFeed()
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
        ),
        new ParallelCommandGroup(
        new ShooterSetAcquireCoral() 
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())).andThen(new ShooterStop()),
        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
            .andThen(
                new ParallelCommandGroup(
                    new SwerveDrivePIDToBranchScore(CoralBranch.D, 4, true)
                        .withTimeout(5)
                        .deadlineFor(new LEDApplyPattern(CoralBranch.D.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                        .andThen(
                            new SuperStructureCoralL4Front()
                                .andThen(new SuperStructureWaitUntilAtTarget())
                        )
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
        new ShooterSetAcquireCoral() 
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())).andThen(new ShooterStop()),
        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
            .andThen(
                new ParallelCommandGroup(
                    new SwerveDrivePIDToBranchScore(CoralBranch.C, 4, true)
                        .withTimeout(5)
                        .deadlineFor(new LEDApplyPattern(CoralBranch.C.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                        .andThen(
                            new SuperStructureCoralL4Front()
                                .andThen(new SuperStructureWaitUntilAtTarget())
                        )
                )
            )
    ),

        // To HP, Score B2
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
        new ShooterSetAcquireCoral() 
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())).andThen(new ShooterStop()),
        new WaitUntilCommand(() -> Shooter.getInstance().hasCoral() || Funnel.getInstance().hasCoral())
            .andThen(
                new ParallelCommandGroup(
                    new SwerveDrivePIDToBranchScore(CoralBranch.B, 2, true)
                        .withTranslationalConstraints(3.25, 5.5)
                        .withTimeout(5)
                        .deadlineFor(new LEDApplyPattern(CoralBranch.B.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                    new WaitUntilCommand(() -> Shooter.getInstance().hasCoral())
                        .andThen(
                            new SuperStructureCoralL2Front()
                                .andThen(new SuperStructureWaitUntilAtTarget())
                        )
                )
            )
    ),
        
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
        )

    );

}

}
