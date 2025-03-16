package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SwerveDrivePIDToBarge extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToBarge() {
        super(() -> Field.getCatapultTargetPose(CommandSwerveDrivetrain.getInstance().getPose()));
    }
}
