
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.ReefFace;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDAssistToClosestL1ShooterReady extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Gamepad driver;
    
    private final VStream driverLinearVelocity;
    private final IStream driverAngularVelocity;

    private ReefFace closestReefFace;

    private final HolonomicController controller;

    private final FieldObject2d targetPose2d;

    public SwerveDrivePIDAssistToClosestL1ShooterReady(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        driverLinearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Drive.MAX_TELEOP_SPEED),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        driverAngularVelocity = IStream.create(driver::getRightX)
            .filtered(
                x -> -x,
                x -> SLMath.deadband(x, Turn.DEADBAND),
                x -> SLMath.spow(x, Turn.POWER),
                x -> x * (Turn.MAX_TELEOP_TURN_SPEED),
                new LowPassFilter(Turn.RC));

        controller = new HolonomicController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFUALT_MAX_ANGULAR_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION)));

        targetPose2d = Field.FIELD2D.getObject("Target Pose");

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void initialize() {
        this.closestReefFace = ReefUtil.getClosestReefFace();
    }

    @Override
    public void execute() {
        Pose2d targetPose = closestReefFace.getL1ShooterClearPose();
        targetPose2d.setPose(Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose));

        controller.update(targetPose, swerve.getPose());

        ChassisSpeeds controllerFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(controller.getOutput(), swerve.getPose().getRotation());

        Rotation2d reefFaceParallelHeading = closestReefFace.getCorrespondingAprilTagPose().getRotation().rotateBy(Rotation2d.kCCW_90deg);
        // double driverVelocityComponentParallelToReefFace = driverLinearVelocity.get().dot(new Vector2D(reefFaceParallelHeading.getCos(), reefFaceParallelHeading.getSin()));
        double driverVelocityComponentParallelToReefFace = driverLinearVelocity.get().y * (closestReefFace.isOnDriverStationSide() ? -1 : 1);
        Vector2D driverVelocityVectorParallelToReefFace = new Vector2D(
            driverVelocityComponentParallelToReefFace * reefFaceParallelHeading.getCos(), 
            driverVelocityComponentParallelToReefFace * reefFaceParallelHeading.getSin());
        
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(controllerFieldRelativeSpeeds.vxMetersPerSecond + driverVelocityVectorParallelToReefFace.x)
            .withVelocityY(controllerFieldRelativeSpeeds.vyMetersPerSecond + driverVelocityVectorParallelToReefFace.y)
            .withRotationalRate(controllerFieldRelativeSpeeds.omegaRadiansPerSecond + driverAngularVelocity.get()));

        SmartDashboard.putNumber("Alignment/Target x", targetPose.getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m per s)", controller.getOutput().vxMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m per s)", controller.getOutput().vyMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad per s)", controller.getOutput().omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        Field.clearFieldObject(targetPose2d);
    }

}