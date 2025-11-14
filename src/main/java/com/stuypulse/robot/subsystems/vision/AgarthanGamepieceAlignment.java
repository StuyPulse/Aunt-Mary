package com.stuypulse.robot.subsystems.vision;

import java.time.OffsetDateTime;

import com.fasterxml.jackson.databind.deser.impl.UnwrappedPropertyHandler;
import com.stuypulse.robot.constants.Cameras;


/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AgarthanGamepieceAlignment extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;
    private final Gamepad driver;

    private final Rotation2d offset;
    private double targetAngle;
    private Rotation2d initialRobot;
    private double test;
    private Rotation2d txnc;

    private final static double kP_VEL_PARALLEL = 10.0;

    private final AngleController angleController;
    private final VStream linearVelocity;

    public AgarthanGamepieceAlignment(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        vision = LimelightVision.getInstance();
        this.driver = driver;
        offset = new Rotation2d(Cameras.LimelightCameras[2].getLocation().getRotation().getZ());
        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(1.0,
                        Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));

        linearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Drive.MAX_TELEOP_SPEED),
                new VRateLimit(1.0),
                new VLowPassFilter(Drive.RC));
                
        addRequirements(swerve, vision);
        targetAngle = 0.0;
    }

    @Override
    public void initialize() {
        initialRobot = swerve.getPose().getRotation();
        txnc = Rotation2d.fromDegrees(vision.getLastGoodFrame().txncOfHighestArea());
        // test = 10.0;
    }

    @Override
    public void execute() {
        // REMEMBER TO CHANGE BELOW LINES BEFORE TESTING ON ROBOT
        // test = (test < 0.1) ? test : test-0.1;
        // Rotation2d txnc = Rotation2d.fromDegrees(vision.getLastGoodFrame().txncOfHighestArea());
        double unWrappedAngle = initialRobot.getDegrees();
        if (unWrappedAngle < 0.0) {
            unWrappedAngle += 360.0;
        } //convert to 0-360

        double targetAngle = ((offset.getDegrees() - txnc.getDegrees()) + unWrappedAngle) % 360;
        if (targetAngle > 180.0) {
            targetAngle -= 360.0;
        }

        double speed_parallel = kP_VEL_PARALLEL * Math.abs(txnc.getRadians());
        Vector2D vel_parallel = new Vector2D(
            Math.cos(Units.degreesToRadians(offset.getDegrees()+ 90.0)),
            Math.sin(Units.degreesToRadians(offset.getDegrees()+ 90.0)))
                .mul(speed_parallel).rotate(Angle.fromDegrees(targetAngle));
        double target_omega = angleController.update(
            Angle.fromDegrees(targetAngle),
            Angle.fromRotation2d(swerve.getPose().getRotation()));

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withRotationalRate(target_omega));

        SmartDashboard.putNumber("Vision/Target Omega", target_omega);

        SmartDashboard.putNumber("Vision/TEST TEST TEST TEST", test);
        SmartDashboard.putNumber("Vision/TXNC Degrees", txnc.getDegrees());
        SmartDashboard.putNumber("Vision/Current Robot Angle", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/Target Robot Angle", targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }
}
