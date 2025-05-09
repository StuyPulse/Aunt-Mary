package com.stuypulse.robot.commands.autons.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;

public class DriveToTest extends SequentialCommandGroup {
    
    public DriveToTest() {
        addCommands(
            new SwerveDrivePIDToPose(new Pose2d(1.0, 1.864, Rotation2d.kZero)),
            new WaitCommand(1)
        );
    }
}
