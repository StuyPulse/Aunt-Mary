/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;

public class ClimbDriveToClimb extends ClimbDriveToAngle {

    public ClimbDriveToClimb() {
        super(Settings.Climb.CLIMBED_ANGLE);
    }
}
