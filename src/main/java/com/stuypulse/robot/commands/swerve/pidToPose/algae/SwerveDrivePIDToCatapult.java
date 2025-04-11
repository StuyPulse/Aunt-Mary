
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToCatapult extends SwerveDrivePIDToPose {
    public SwerveDrivePIDToCatapult(Supplier<Double> targetYDistanceFromCenter) {
        super(() -> Field.getCatapultTargetPose(targetYDistanceFromCenter.get()));
    }

    public SwerveDrivePIDToCatapult(double targetYDistanceFromCenter) {
        this(() -> targetYDistanceFromCenter);
    }
}
