package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToClosestL1FroggyReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToClosestL1FroggyReady() {
        super(() -> ReefUtil.getClosestReefFace().getL1FroggyClearPose());
    }
}