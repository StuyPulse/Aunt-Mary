package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;

public class ClimbDriveToStow extends ClimbDriveToAngle {

    public ClimbDriveToStow() {
        super(Settings.Climb.STOW_ANGLE);
    }
    
}
