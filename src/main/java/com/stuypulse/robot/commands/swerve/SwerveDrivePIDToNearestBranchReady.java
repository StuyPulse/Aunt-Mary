package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToNearestBranchReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranchReady(boolean isScoringFrontSide) {
        super(() -> Field.getClosestBranch().getReadyPose(isScoringFrontSide));
    }
}
