package com.stuypulse.robot.commands.autons.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.subsystems.swerve.*;

public class DriveTest extends SequentialCommandGroup{
    public DriveTest(){
        addCommands(
            new SwerveDrivePIDToPose(new Pose2d (4.572, 0, Rotation2d.kZero))
        );
    }
}
