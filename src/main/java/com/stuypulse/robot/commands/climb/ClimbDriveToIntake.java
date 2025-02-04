package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;

public class ClimbDriveToIntake extends ClimbDriveToAngle {

    public ClimbDriveToIntake() {
        super(Settings.Climb.INTAKE_ANGLE);
    }
    
}
