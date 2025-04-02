
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

public class SwerveDriveCoralScoreAlignWithClearance extends ParallelCommandGroup {

    private enum Mode {
        CLEAR,
        SCORE
    }

    private Mode mode;
    private SuperStructureState correspondingSuperStructureState;
    private Supplier<CoralBranch> branch;
    private boolean isScoringFrontSide;
    private int level;

    public  SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState) {
        this.mode = Mode.CLEAR;
        this.correspondingSuperStructureState = correspondingSuperStructureState;
        this.branch = branch;
        this.isScoringFrontSide = isScoringFrontSide;
        this.level = level;

        addCommands(
            new WaitUntilCommand(this::isClear).andThen(getSwitchToScoreCommand()),
            new SwerveDrivePIDToPose(this::getTargetPose)
                .withCanEnd(() -> mode == Mode.SCORE),
            new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)
        );
    }

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState, double maxVel, double maxAccel) {
        this.mode = Mode.CLEAR;
        this.correspondingSuperStructureState = correspondingSuperStructureState;
        this.branch = branch;
        this.isScoringFrontSide = isScoringFrontSide;
        this.level = level;

        addCommands(
            new WaitUntilCommand(this::isClear).andThen(getSwitchToScoreCommand()),
            new SwerveDrivePIDToPose(this::getTargetPose)
                .withTranslationalConstraints(maxVel, maxAccel)
                .withCanEnd(() -> mode == Mode.SCORE),
            new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)
        );
    } 

    private Pose2d getTargetPose() {
        if (mode == Mode.CLEAR) {
            return branch.get().getClearancePose(isScoringFrontSide);
        }
        else {
            return branch.get().getScorePose(level, isScoringFrontSide);
        }
    }

    private boolean canEnd() {
        return mode == Mode.SCORE;
    }

    private InstantCommand getSwitchToScoreCommand() {
        return new InstantCommand(() -> this.mode = Mode.SCORE);
    }

    private boolean isClear() {
        return SuperStructure.getInstance().getState() == correspondingSuperStructureState
            && SuperStructure.getInstance().canSkipClearance();
    }
}
