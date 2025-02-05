/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbDriveToAngle extends Command {
    private Climb climb;
    private final double targetDegrees;

    public ClimbDriveToAngle(double targetDegrees) {
        Climb climb = Climb.getInstance();
        addRequirements(climb);
        this.targetDegrees = targetDegrees;
    }

    public void initialize() {
        climb.setTargetDegrees(targetDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climb.getAngle().getDegrees() - climb.getTargetAngle().getDegrees()) <= Settings.Climb.CLIMB_ANGLE_TOLERANCE;
    }
}
