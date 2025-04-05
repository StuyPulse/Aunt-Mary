
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.robot.util.ReefUtil.ReefFace;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

public class ScoreRoutine extends SequentialCommandGroup {
    private final SuperStructure superStructure;
    private Supplier<CoralBranch> targetBranch;
    
    private ReefFace targetReefFace; // Used for driver input

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, ReefUtil::getClosestCoralBranch);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch) {
        this(level, isFrontFacingReef, () -> targetBranch);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Supplier<CoralBranch> targetBranch) {
        superStructure = SuperStructure.getInstance();
        this.targetBranch = targetBranch;

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, correspondingSuperStructureState)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new SuperStructureSetState(correspondingSuperStructureState))
                        .onlyIf(() -> superStructure.getState() != correspondingSuperStructureState)
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
                .alongWith(new ShooterAcquireCoral()),
            new SuperStructureWaitUntilAtTarget(), // Re-check positioning
            Shooter.getCorrespondingShootCommand(level, isFrontFacingReef),
            new LEDApplyPattern(Settings.LED.SCORE_COLOR)
        );
    }

    public ScoreRoutine(Gamepad driver, int level, boolean isFrontFacingReef) {
        superStructure = SuperStructure.getInstance();
        this.targetReefFace = ReefFace.AB; // Set default value to avoid null pointer exception
        this.targetBranch = getCoralBranchSupplierWithDriverInput(driver);

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);

        addCommands(
            resetReefFaceToClosestReefFace(),
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, correspondingSuperStructureState)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new SuperStructureSetState(correspondingSuperStructureState))
                        .onlyIf(() -> superStructure.getState() != correspondingSuperStructureState)
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
                .alongWith(new ShooterAcquireCoral()),
            new SuperStructureWaitUntilAtTarget(), // Re-check positioning
            Shooter.getCorrespondingShootCommand(level, isFrontFacingReef),
            new LEDApplyPattern(Settings.LED.SCORE_COLOR)
        );

        mapReefFaceOffsetToController(driver);
    }

    private InstantCommand resetReefFaceToClosestReefFace() {
        return new InstantCommand(() -> this.targetReefFace = ReefUtil.getClosestReefFace());
    }

    private void mapReefFaceOffsetToController(Gamepad driver) {
        driver.getLeftBumper().onTrue(new InstantCommand(() -> this.targetReefFace = this.targetReefFace.rotateCCW(-1)));
        driver.getRightBumper().onTrue(new InstantCommand(() -> this.targetReefFace = this.targetReefFace.rotateCCW(1)));
    }

    private Supplier<CoralBranch> getCoralBranchSupplierWithDriverInput(Gamepad driver) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return targetReefFace.getRightBranchFieldRelative();
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return targetReefFace.getLeftBranchFieldRelative();
            }
            else {
                return targetReefFace.getClosestCoralBranch();
            }
        };
    }
}
