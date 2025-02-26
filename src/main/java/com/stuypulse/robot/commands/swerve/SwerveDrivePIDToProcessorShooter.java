package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToProcessorShooter extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToProcessorShooter() {
        super(Field.getTargetPoseForProcessorShooter());
    }
}
