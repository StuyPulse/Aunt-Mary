package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;

public class ClimbDriveToAcquired extends ClimbDriveToAngle {

    public ClimbDriveToAcquired() {
        super(Settings.Climb.ACQUIRED_ANGLE);
    }
    
}

