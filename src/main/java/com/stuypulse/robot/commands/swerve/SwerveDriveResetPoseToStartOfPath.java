
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.PathUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class SwerveDriveResetPoseToStartOfPath extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private final PathPlannerPath path;

    public SwerveDriveResetPoseToStartOfPath(PathPlannerPath path) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.path = path;
    }

    public SwerveDriveResetPoseToStartOfPath(String path) {
        this(PathUtil.load(path));
    }

    @Override
    public void initialize() {
        swerve.resetPose(path.getStartingDifferentialPose());
    }
}
