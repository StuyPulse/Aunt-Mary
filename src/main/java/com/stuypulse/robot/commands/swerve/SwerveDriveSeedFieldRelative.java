
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveSeedFieldRelative extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;

    public SwerveDriveSeedFieldRelative() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.seedFieldCentric();
    }
}
