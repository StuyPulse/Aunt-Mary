package com.stuypulse.robot.subsystems.vision;

/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import java.util.Queue;

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ServoToGamepiece extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision limelightVision;
    private final Rotation2d cameraAngle;
    private final Queue<ServoObjectData> fifo;
    private final AngleController angleController;

    public ServoToGamepiece(Rotation2d cameraAngle) {
        swerve = CommandSwerveDrivetrain.getInstance();
        // get the raw detections, and then take the last actual detection's txnc as the
        // target angle
        limelightVision = LimelightVision.getInstance();
        this.cameraAngle = cameraAngle;
        this.fifo = limelightVision.getNeuralNetworkFIFO();

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY,
                        Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));
        addRequirements(swerve);
    }

    // still figure out what to do if the last frame is null?
    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
                .withRotationalRate(angleController.update(
                        Angle.fromRotation2d(cameraAngle.minus(new Rotation2d(fifo.poll().getObjectAngle()))),
                        Angle.fromRotation2d(swerve.getPose().getRotation()))));
        // implement the driving part
    }
}
