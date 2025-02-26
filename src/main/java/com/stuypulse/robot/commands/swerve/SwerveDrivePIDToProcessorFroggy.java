package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToProcessorFroggy extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToProcessorFroggy() {
        super(Field.getTargetPoseForProcessorFroggy());
    }
}
