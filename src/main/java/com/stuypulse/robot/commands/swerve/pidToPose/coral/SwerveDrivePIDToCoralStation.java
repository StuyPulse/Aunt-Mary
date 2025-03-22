
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToCoralStation extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToCoralStation(boolean isCD) {
        super(() -> Field.CoralStation.getCoralStation(isCD).getTargetPose());
    }
}