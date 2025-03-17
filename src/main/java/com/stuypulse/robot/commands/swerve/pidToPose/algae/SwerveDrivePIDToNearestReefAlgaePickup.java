package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestReefAlgaePickup extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestReefAlgaePickup(boolean isFrontFacingReef) {
        super(() -> ReefUtil.getClosestAlgae().getTargetPose(isFrontFacingReef));
        withoutMotionProfile();
    }
}
