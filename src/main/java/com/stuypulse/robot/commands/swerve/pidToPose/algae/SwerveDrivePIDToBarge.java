
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SwerveDrivePIDToBarge extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToBarge(boolean in) {
        super(() -> Field.getCatapultTargetPoseA(CommandSwerveDrivetrain.getInstance().getPose(), in));
    }
}
