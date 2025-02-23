package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToCurrentNearestBranchReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToCurrentNearestBranchReady(boolean isScoringFrontSide) {
        super(() -> Field.getClosestBranch().getReadyPose(isScoringFrontSide));
    }
}
