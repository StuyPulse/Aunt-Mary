package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestReefAlgaeReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestReefAlgaeReady(boolean isFrontFacingReef) {
        super(() -> ReefUtil.getClosestAlgae().getReadyPose(isFrontFacingReef));
    }
}
