package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field.CoralBranch;

public class SwerveDrivePIDToBranchReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchReady(boolean isScoringFrontSide, CoralBranch nearestBranch) {
        super(() -> nearestBranch.getReadyPose(isScoringFrontSide));
    }
}
