package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SwerveDriveWaitUntilClearFromBarge118 extends WaitUntilCommand{
    public SwerveDriveWaitUntilClearFromBarge118() {
        super(() -> Math.abs(CommandSwerveDrivetrain.getInstance().getPose().getX() - Field.LENGTH / 2) 
                    > Settings.Clearances.CLEARANCE_DISTANCE_FROM_CENTERLINE_BARGE_118 - Settings.Swerve.Alignment.Tolerances.X_TOLERANCE.get());
    }
}
