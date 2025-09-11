
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.ReefFace;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDAssistToL1Froggy extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Gamepad driver;
    
    private final VStream driverLinearVelocity;

    private final HolonomicController controller;

    private final FieldObject2d targetPose2d;
    private final int level;

    private Supplier<Pose2d> targetPose;

    public SwerveDrivePIDAssistToL1Froggy(Gamepad driver, int level) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;
        this.level = level;

        driverLinearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Drive.MAX_TELEOP_SPEED),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        controller = new HolonomicController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD).add(new MotorFeedforward(0, 0, 0).position()),
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD).add(new MotorFeedforward(0, 0, 0).position()),
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
        targetPose = () -> ReefUtil.getClosestReefFace().getL1FroggyScorePose(level);
    }

    @Override
    public void execute() {
        Pose2d currentTarget = targetPose.get();

        ReefFace reefFace = ReefUtil.getClosestReefFace();
        Pose2d reefCenter = reefFace.getL1FroggyScorePose(level);
        
        Pose2d leftBound  = reefCenter.transformBy(
            new Transform2d(new Translation2d(Field.LENGTH_OF_REEF_FACE / 2.0, 0), new Rotation2d()));
        Pose2d rightBound  = reefCenter.transformBy(
            new Transform2d(new Translation2d(-Field.LENGTH_OF_REEF_FACE / 2.0, 0), new Rotation2d()));

        Translation2d L = leftBound.getTranslation();
        Translation2d R = rightBound.getTranslation();
        Translation2d robot = swerve.getPose().getTranslation();
        Translation2d reef = R.minus(L);
        Translation2d leftToRobot = robot.minus(L);
        
        double magSquared = reef.getX() * reef.getX() + reef.getY() * reef.getY();

        double leftToRobot_dot_reef = leftToRobot.getX() * reef.getX() + leftToRobot.getY() * reef.getY();
        double projected_t = leftToRobot_dot_reef / magSquared;

        double dt = (reefFace == ReefFace.EF || reefFace == ReefFace.GH || reefFace == ReefFace.IJ) ? 
            driverLinearVelocity.get().x / Field.LENGTH_OF_REEF_FACE : -driverLinearVelocity.get().x / Field.LENGTH_OF_REEF_FACE;
        
        double new_t = MathUtil.clamp(projected_t + dt, 0.0, 1.0);
        
        double shift_dist = (new_t - projected_t) * Field.LENGTH_OF_REEF_FACE;
        
        Pose2d newPose = currentTarget.transformBy(
            new Transform2d(new Translation2d(shift_dist, 0.0), new Rotation2d())
        );

        controller.update(newPose, swerve.getPose());
        targetPose2d.setPose(Robot.isBlue() ? newPose : Field.transformToOppositeAlliance(newPose));

        ChassisSpeeds controllerFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(controller.getOutput(), swerve.getPose().getRotation());
        
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(controllerFieldRelativeSpeeds.vxMetersPerSecond)
            .withVelocityY(controllerFieldRelativeSpeeds.vyMetersPerSecond)
            .withRotationalRate(controllerFieldRelativeSpeeds.omegaRadiansPerSecond));

        SmartDashboard.putNumber("Alignment/Target x", targetPose.get().getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.get().getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.get().getRotation().getDegrees());

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