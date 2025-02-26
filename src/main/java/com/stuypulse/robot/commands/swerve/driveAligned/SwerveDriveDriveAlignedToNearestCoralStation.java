package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.stuylib.input.Gamepad;

public class SwerveDriveDriveAlignedToNearestCoralStation extends SwerveDriveDriveAligned{
    
    public SwerveDriveDriveAlignedToNearestCoralStation(Gamepad driver) {
        super(driver, () -> Field.getClosestCoralStationTargetPose().getRotation());
    }
}
