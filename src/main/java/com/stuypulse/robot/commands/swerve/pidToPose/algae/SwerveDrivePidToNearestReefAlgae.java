package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePidToNearestReefAlgae extends SwerveDrivePIDToPose{
    public SwerveDrivePidToNearestReefAlgae() {
        super(() -> ReefUtil.getClosestAlgae().getTargetPose());
    }
}
