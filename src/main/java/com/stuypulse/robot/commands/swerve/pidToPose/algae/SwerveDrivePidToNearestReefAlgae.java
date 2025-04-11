
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDrivePidToNearestReefAlgae extends SequentialCommandGroup {
    public SwerveDrivePidToNearestReefAlgae(Supplier<Boolean> isFrontFacingReef) {
        addCommands(
            new SwerveDrivePIDToNearestReefAlgaeReady(isFrontFacingReef.get()),
            new SwerveDrivePIDToNearestReefAlgaePickup(isFrontFacingReef.get())
        );
    }
}
