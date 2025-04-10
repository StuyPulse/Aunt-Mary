
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToClosestL1FroggyScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToClosestL1FroggyScore() {
        super(() -> ReefUtil.getClosestReefFace().getL1FroggyScorePose());
        super.withTolerance(
            Settings.Swerve.Alignment.Tolerances.X_TOLERANCE_FROGGY, 
            Settings.Swerve.Alignment.Tolerances.Y_TOLERANCE_FROGGY,
            Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_FROGGY);
    }
}
