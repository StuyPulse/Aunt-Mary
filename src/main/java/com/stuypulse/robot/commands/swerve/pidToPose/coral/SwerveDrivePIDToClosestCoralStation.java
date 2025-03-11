package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToClosestCoralStation extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToClosestCoralStation() {
        super(() -> Field.CoralStation.getClosestCoralStation().getTargetPose());
    }
}