package com.stuypulse.robot.commands.autons.tests;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveBackwardTest extends SequentialCommandGroup {
    public DriveBackwardTest() {
        Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getPose();
        Pose2d targetPose = new Pose2d(
            currentPose.getX() - 1.542,
            currentPose.getY(),
            Rotation2d.kZero
        );

        addCommands(
            new SwerveDrivePIDToPose(() -> targetPose).withCanEnd(() -> true) // static pose
        );
        
        addRequirements(CommandSwerveDrivetrain.getInstance());
    }
}
