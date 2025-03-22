
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDrivePidToNearestReefAlgae extends SequentialCommandGroup {
    public SwerveDrivePidToNearestReefAlgae(boolean isFrontFacingReef) {
        addCommands(
            new SwerveDrivePIDToNearestReefAlgaeReady(isFrontFacingReef),
            new SwerveDrivePIDToNearestReefAlgaePickup(isFrontFacingReef)
        );
    }
}
