
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
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

public class SwerveDriveCoralScoreAlignWithClearance extends SequentialCommandGroup {

    private enum Mode {
        CLEAR,
        SCORE
    }

    private Mode mode;
    private BStream canEnd;
    private SuperStructureState correspondingSuperStructureState;
    private Supplier<CoralBranch> branch;
    private boolean isScoringFrontSide;
    private int level;

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState) {
        this(branch, level, isScoringFrontSide, correspondingSuperStructureState, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION);
    }

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState, double maxVel, double maxAccel) {
        this.mode = Mode.CLEAR;
        this.correspondingSuperStructureState = correspondingSuperStructureState;
        this.branch = branch;
        this.isScoringFrontSide = isScoringFrontSide;
        this.level = level;

        canEnd = BStream.create(() -> this.mode == Mode.SCORE).filtered(new BDebounce.Rising(0.5));

        addCommands(
            new InstantCommand(() -> this.mode = Mode.CLEAR),
            new WaitUntilCommand(this::isClear).andThen(new InstantCommand(() -> this.mode = Mode.SCORE))
                .alongWith(new SwerveDrivePIDToPose(this::getTargetPose)
                    .withoutMotionProfile()
                    // .withTranslationalConstraints(maxVel, maxAccel)
                    .withCanEnd(canEnd::get)
                    .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)))
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

    private boolean isClear() {
        return SuperStructure.getInstance().getState() == correspondingSuperStructureState
            && SuperStructure.getInstance().canSkipClearance()
            && branch.get() == ReefUtil.getClosestCoralBranch();
    }
}
