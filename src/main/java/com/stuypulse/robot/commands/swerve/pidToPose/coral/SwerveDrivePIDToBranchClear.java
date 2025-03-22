
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import java.util.function.Supplier;

public class SwerveDrivePIDToBranchClear extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchClear(Supplier<CoralBranch> branch, boolean isScoringFrontSide) {
        super(() -> branch.get().getClearancePose(isScoringFrontSide));
    }

    public SwerveDrivePIDToBranchClear(CoralBranch branch, boolean isScoringFrontSide) {
        this(() -> branch, isScoringFrontSide);
    }
}
