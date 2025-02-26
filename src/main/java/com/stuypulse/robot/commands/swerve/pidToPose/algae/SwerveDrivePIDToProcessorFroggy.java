package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToProcessorFroggy extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToProcessorFroggy() {
        super(Field.getTargetPoseForProcessorFroggy());
    }
}
