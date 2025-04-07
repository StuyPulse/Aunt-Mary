
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignAuton;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

public class TempScoreRoutineL2 extends SequentialCommandGroup {
    private final SuperStructure superStructure;

    public TempScoreRoutineL2(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, ReefUtil::getClosestCoralBranch);
    }

    public TempScoreRoutineL2(int level, boolean isFrontFacingReef, CoralBranch targetBranch) {
        this(level, isFrontFacingReef, () -> targetBranch);
    }

    public TempScoreRoutineL2(int level, boolean isFrontFacingReef, Supplier<CoralBranch> targetBranch) {
        superStructure = SuperStructure.getInstance();

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);

        addCommands(
            new SwerveDrivePIDToBranchScore(targetBranch, level, isFrontFacingReef)
                .alongWith(
                    new ShooterWaitUntilHasCoral()
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

    public TempScoreRoutineL2(Gamepad driver, int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, getCoralBranchSupplierWithDriverInput(driver));
    }

    private static Supplier<CoralBranch> getCoralBranchSupplierWithDriverInput(Gamepad driver) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.getClosestReefFace().getRightBranchFieldRelative();
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.getClosestReefFace().getLeftBranchFieldRelative();
            }
            else {
                return ReefUtil.getClosestCoralBranch();
            }
        };
    }
}
