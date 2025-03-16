package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToCoralStation extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToCoralStation(boolean isCD) {
        super(() -> Field.CoralStation.getCoralStation(isCD).getTargetPose());
    }
}