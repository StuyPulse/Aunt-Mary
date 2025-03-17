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
