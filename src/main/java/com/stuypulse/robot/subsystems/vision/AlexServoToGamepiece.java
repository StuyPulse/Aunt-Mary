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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlexServoToGamepiece extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;
    private final Gamepad driver;

    private final Rotation2d offset;
    private double angleOfGamepiece;
    private Rotation2d robotHeading;
    private double test;

    private final AngleController angleController;
    private final VStream linearVelocity;

    public AlexServoToGamepiece(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        vision = LimelightVision.getInstance();
        this.driver = driver;
        offset = new Rotation2d(Cameras.LimelightCameras[2].getLocation().getRotation().getZ());
        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY,
                        Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));

        linearVelocity = VStream.create(this::getDriverInputAsVelocity)
                .filtered(
                        new VDeadZone(Drive.DEADBAND),
                        x -> x.clamp(1),
                        x -> x.pow(Drive.POWER),
                        x -> x.mul(Drive.MAX_TELEOP_SPEED),
                        new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                        new VLowPassFilter(Drive.RC));

        addRequirements(swerve, vision);
        angleOfGamepiece = 0.0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Rotation2d txnc = Rotation2d.fromDegrees(test);
        robotHeading = swerve.getPose().getRotation();
        // need to query this on every iteration
        Rotation2d txnc = Rotation2d.fromDegrees(vision.getLastGoodFrame().txncOfHighestArea());

        double unWrappedAngle = robotHeading.getDegrees();
        if (unWrappedAngle < 0.0) {
            unWrappedAngle += 360.0;
        } // convert to 0-360

        angleOfGamepiece = ((offset.getDegrees() - txnc.getDegrees()) + unWrappedAngle) % 360;
        // target angle but needs to be converted in terms of swerve pose
        // froggy at 0 -> shooter at 90
        // froggy at 90 shooter at 180 or -180
        // froggy at 180 shooter at -90
        // froggy at -90 shooter at 0
        // offset = 275 - 15 + 260 -> 160
        // 160 is angle of the gamepiece, shooter needs to be at -115
        double swerveTargetAngle = angleOfGamepiece + 85.0; // magic number to account for shooter heading
        if (swerveTargetAngle > 180) {
            swerveTargetAngle -= 360;
        }

        // this gives us the field-relative angle of the gamepiece
        // we need to transform this such that this is the angle the shooter must be to
        // have the froggy face this angle
        // in other words, if we directly PID to this angle, the shooter will face the
        // game piece

        // convert back to swerve pose
        // the problem is that the swerve wraps from 0 to 180 to -180 to 0
        // if we had shooter facing 0 degrees, the froggy side is actually at -90
        // lets say the coral is ten degrees to the left of the camera, representing a
        // -10 degree angle
        // this means the target angle of the swerve is -80 degrees
        // in other words, we need to subtract txnc not add!
        // in the case where the swerve is at 90 degrees and the coral is again 10
        // degrees left, the target angle is 10 degrees
        // we assume querying swerve.getpose does not meaningfully change in the time it
        // takes to execute..

        double final_target = angleController.update(
                Angle.fromDegrees(angleOfGamepiece),
                Angle.fromRotation2d(swerve.getPose().getRotation()));

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
                .withVelocityX(linearVelocity.get().x)
                .withVelocityY(linearVelocity.get().y)
                .withRotationalRate(swerveTargetAngle));

        SmartDashboard.putNumber("Vision/Final Target", final_target);

        SmartDashboard.putNumber("Vision/camera offset", offset.getDegrees());
        SmartDashboard.putNumber("Vision/TEST TEST TEST TEST", test);
        SmartDashboard.putNumber("Vision/TXNC Degrees", txnc.getDegrees());
        SmartDashboard.putNumber("Vision/Current Robot Angle", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/Target Robot Angle", swerveTargetAngle);
        // swerve.setControl(swerve.getFieldCentricSwerveRequest()
        // .withRotationalRate(angleController.update(
        // Angle.fromRotation2d(cameraAngle.minus(targetAngle)),
        // Angle.fromRotation2d(swerve.getPose().getRotation().plus(new
        // Rotation2d(Math.PI/2.0)))))); // either plus or minus
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
