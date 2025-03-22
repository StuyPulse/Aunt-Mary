
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestReefAlgaeReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestReefAlgaeReady(boolean isFrontFacingReef) {
        super(() -> ReefUtil.getClosestAlgae().getReadyPose(isFrontFacingReef));
    }
}
