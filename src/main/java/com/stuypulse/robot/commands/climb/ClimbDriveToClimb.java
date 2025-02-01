package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;

public class ClimbDriveToClimb extends ClimbDriveToAngle {

    public ClimbDriveToClimb() {
        super(Settings.Climb.CLIMBED_ANGLE);
    }
    
}
