package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field.CoralBranch;

public class SwerveDrivePIDToNearestBranchReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranchReady(boolean isScoringFrontSide, CoralBranch nearestBranch) {
        super(() -> nearestBranch.getReadyPose(isScoringFrontSide));
    }
}
