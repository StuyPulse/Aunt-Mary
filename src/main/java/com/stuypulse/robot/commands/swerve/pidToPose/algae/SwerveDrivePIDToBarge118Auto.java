
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToBarge118Auto extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToBarge118Auto(Supplier<Double> targetYDistanceFromCenter) {
        super(() -> Field.getCatapultTargetPoseAuton(targetYDistanceFromCenter.get()));
    }

    public SwerveDrivePIDToBarge118Auto(double targetYDistanceFromCenter) {
        this(() -> targetYDistanceFromCenter);
    }
}
