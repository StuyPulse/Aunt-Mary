
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.driveAligned;

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

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class SwerveDriveDriveAligned extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Gamepad driver;

    private final VStream linearVelocity;

    private final Supplier<Rotation2d> targetAngle;
    private final AngleController angleController;

    public SwerveDriveDriveAligned(Gamepad driver, Supplier<Rotation2d> targetAngle) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        this.targetAngle = targetAngle;

        linearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Drive.MAX_TELEOP_SPEED),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));
                
        addRequirements(swerve);
    }

    public SwerveDriveDriveAligned(Gamepad driver, Rotation2d targetAngle) {
        this(driver, () -> targetAngle);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(linearVelocity.get().x)
            .withVelocityY(linearVelocity.get().y)
            .withRotationalRate(angleController.update(
                Angle.fromRotation2d(targetAngle.get()),
                Angle.fromRotation2d(swerve.getPose().getRotation()))));
    }
}