
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestReefAlgaePickup extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestReefAlgaePickup(boolean isFrontFacingReef) {
        super(() -> ReefUtil.getClosestAlgae().getTargetPose(isFrontFacingReef));
        super.withTolerance(
            Settings.Swerve.Alignment.Tolerances.X_TOLERANCE_REEF_PICKUP, 
            Settings.Swerve.Alignment.Tolerances.Y_TOLERANCE_REEF_PICKUP,
            Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_REEF_PICKUP);
        withoutMotionProfile();
    }
}
