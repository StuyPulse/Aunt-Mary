package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;


/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ServoToGamepiece extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;
    private final AngleController angleController;

    public ServoToGamepiece() {
        swerve = CommandSwerveDrivetrain.getInstance();
        vision = LimelightVision.getInstance();
        new Rotation2d(Cameras.LimelightCameras[2].getLocation().getRotation().getZ());
        new Rotation2d();
        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY,
                        Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));
        addRequirements(swerve, vision);
    }

    @Override
    public void execute() {
        // txnc is relative to camera, and camera is 275 relative to robot front => 275 + txnc
        // but we're aligning froggy to the gamepiece, which is 270 relative to robot front
        // so our target is actually (275 + txnc) - 270 = 5 + txnc
        Rotation2d txnc = Rotation2d.fromDegrees(vision.getLastGoodFrame().txncOfHighestArea());
        Rotation2d angleSetpoint = swerve.getPose().getRotation().plus(txnc.plus(Rotation2d.fromDegrees(5)));                                                                                                                                                                                                                                                                                                                                 
    
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withRotationalRate(angleController.update(
                Angle.fromRotation2d(angleSetpoint),
                Angle.fromRotation2d(swerve.getPose().getRotation()))));

        SmartDashboard.putNumber("Vision/TXNC Degrees", txnc.getDegrees());
        SmartDashboard.putNumber("Vision/Current Robot Angle", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/Target Robot Angle", angleSetpoint.getDegrees());
        SmartDashboard.putNumber("Vision/Angle Error", angleSetpoint.minus(swerve.getPose().getRotation()).getDegrees());

        // swerve.setControl(swerve.getFieldCentricSwerveRequest()
        //     .withRotationalRate(angleController.update(
        //         Angle.fromRotation2d(cameraAngle.minus(targetAngle)),
        //         Angle.fromRotation2d(swerve.getPose().getRotation().plus(new Rotation2d(Math.PI/2.0)))))); // either plus or minus
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(cameraAngle.minus(targetAngle).getDegrees() - swerve.getPose().getRotation().plus(new Rotation2d(Math.PI/2.0)).getDegrees()) > Settings.Swerve.Alignment.Tolerances.AUTO_ACQUIRE_TOLERANCE_DEG;
    // }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
