package com.stuypulse.robot.subsystems.vision;
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class ServoToGamepiece extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Rotation2d cameraAngle;
    private final Supplier<Rotation2d> targetAngle;
    private final AngleController angleController;

    public ServoToGamepiece(Supplier<RawDetection[]> rawDetections, Rotation2d cameraAngle) {
        swerve = CommandSwerveDrivetrain.getInstance();
        //get the raw detections, and then take the last actual detection's txnc as the target angle
        this.targetAngle = () -> new Rotation2d (rawDetections.get()[rawDetections.get().length].txnc);
        this.cameraAngle = cameraAngle;

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));
        addRequirements(swerve);
    }


    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withRotationalRate(angleController.update(
                Angle.fromRotation2d(cameraAngle.minus(targetAngle.get())),
                Angle.fromRotation2d(swerve.getPose().getRotation()))));
                //implement the driving part
    }
}