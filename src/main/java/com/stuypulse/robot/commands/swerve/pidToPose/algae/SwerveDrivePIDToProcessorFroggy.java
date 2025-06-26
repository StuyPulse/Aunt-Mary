
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToProcessorFroggy extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToProcessorFroggy() {
        super(Field.getTargetPoseForProcessorFroggy());
    }
}
