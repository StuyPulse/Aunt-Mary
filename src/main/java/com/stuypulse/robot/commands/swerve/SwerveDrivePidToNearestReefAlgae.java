package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePidToNearestReefAlgae extends SwerveDrivePIDToPose{
    public SwerveDrivePidToNearestReefAlgae() {
        super(() -> ReefUtil.getClosestAlgae().getTargetPose());
    }
}
