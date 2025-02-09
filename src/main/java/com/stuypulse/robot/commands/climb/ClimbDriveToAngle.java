/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimbDriveToAngle extends InstantCommand {
    private final Climb climb;
    private final double targetDegrees;

    public ClimbDriveToAngle(double targetDegrees) {
        climb = Climb.getInstance();
        this.targetDegrees = targetDegrees;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setTargetAngle(Rotation2d.fromDegrees(targetDegrees));
    }
}
