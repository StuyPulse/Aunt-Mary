package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToProcessorShooter extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToProcessorShooter() {
        super(Field.getTargetPoseForProcessorShooter());
    }
}
