package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

public class SwerveDrivePIDToBranchClear extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchClear(Supplier<CoralBranch> branch, boolean isScoringFrontSide) {
        super(() -> branch.get().getClearancePose(isScoringFrontSide));
    }

    public SwerveDrivePIDToBranchClear(CoralBranch branch, boolean isScoringFrontSide) {
        this(() -> branch, isScoringFrontSide);
    }
}
