
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import java.util.function.Supplier;

public class SwerveDrivePIDToBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchScore(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide) {
        super(() -> branch.get().getScorePose(level, isScoringFrontSide));
    }

    public SwerveDrivePIDToBranchScore(CoralBranch branch, int level, boolean isScoringFrontSide) {
        this(() -> branch, level, isScoringFrontSide);
    }
}
