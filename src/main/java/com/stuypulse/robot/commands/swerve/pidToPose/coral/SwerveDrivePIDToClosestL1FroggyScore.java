package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToClosestL1FroggyScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToClosestL1FroggyScore() {
        super(() -> ReefUtil.getClosestReefFace().getL1FroggyScorePose());
    }
}
