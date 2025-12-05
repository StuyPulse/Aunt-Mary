package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Cameras;

/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.ServoObjectData;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlexServoToGamepiece extends Command {
    /* TODO (12/5/25)
     * - Test PID gain switching for rotation
     *     Expected Behavior: rotates aggressively from further distances and
     *     normalizes as it gets closer to the gamepiece
     * - Test scaling of forward velocity
     *     Expected Behavior: travels at normal speed from further distances
     *     but slows down as it gets closer to gamepiece
     * Tune the constants at the top of the file to test!
     */

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;
    private final Gamepad driver;

    private final Rotation2d offset;
    private double angleOfGamepiece;
    private Rotation2d robotHeading;

    private final AnglePIDController angleController;
    private final VStream linearVelocity;
    private final IStream angularVelocity;

    private final static double kP_VEL_PARALLEL = 0.5;
    private final static double kP_VEL_FORWARD = 2.5;
    private final static double PID_SCALING = 20.0; // < 20% of frame = faster rotation
    private final static double FORWARD_VEL_SCALING = 20.0; // > 20% of frame = slow down translation

    public AlexServoToGamepiece(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        vision = LimelightVision.getInstance();
        this.driver = driver;
        offset = new Rotation2d(Cameras.LimelightCameras[2].getLocation().getRotation().getZ());

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD);
        angleController.setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY,
                        Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));

        linearVelocity = VStream.create(this::getDriverInputAsVelocity)
                .filtered(
                        new VDeadZone(Drive.DEADBAND),
                        x -> x.clamp(1),
                        x -> x.pow(Drive.POWER),
                        x -> x.mul(Drive.MAX_TELEOP_SPEED),
                        new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                        new VLowPassFilter(Drive.RC));

        angularVelocity = IStream.create(driver::getRightX)
            .filtered(
                x -> -x,
                x -> SLMath.deadband(x, Turn.DEADBAND),
                x -> SLMath.spow(x, Turn.POWER),
                x -> x * (Climb.getInstance().getState() == ClimbState.CLOSED ? Turn.MAX_TELEOP_TURN_SPEED : Turn.MAX_TELEOP_TURN_SPEED_WHILE_CLIMBING),
                new LowPassFilter(Turn.RC));

        addRequirements(swerve, vision);
        angleOfGamepiece = 0.0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ServoObjectData frame = vision.getLastGoodFrame();

        if (frame != null) {
            robotHeading = swerve.getPose().getRotation();
            Rotation2d froggyHeading = robotHeading.minus(Rotation2d.fromDegrees(90.0));
            Rotation2d txnc = Rotation2d.fromDegrees(frame.txncOfHighestArea());
            double ta = frame.getHighestArea();
            double scaleFactor = 1.0;
            if (ta > 5.0) {
                scaleFactor = MathUtil.clamp(Math.max(1.0, PID_SCALING / ta), 1.0, 1.5);
            }
            
            angleController.setP(Alignment.THETA.kP * scaleFactor);
            angleController.setI(Alignment.THETA.kI);
            angleController.setD(Alignment.THETA.kD * Math.sqrt(scaleFactor));

            double unWrappedAngle = robotHeading.getDegrees();
            if (unWrappedAngle < 0.0) {
                unWrappedAngle += 360.0;
            } // convert to 0-360

            angleOfGamepiece = ((offset.getDegrees() - txnc.getDegrees()) + unWrappedAngle) % 360; // target angle but needs to be converted in terms of swerve pose

            double swerveTargetAngle = angleOfGamepiece + 85.0; // magic number to account for shooter heading relative to the gamepiece
            if (swerveTargetAngle > 180) {
                swerveTargetAngle -= 360;
            }

            double speed_parallel = kP_VEL_PARALLEL * Math.abs(txnc.getRadians());
            Vector2D vel_parallel = new Vector2D(
                Math.cos(Units.degreesToRadians(offset.getDegrees()+ 90.0)),
                Math.sin(Units.degreesToRadians(offset.getDegrees()+ 90.0)))
                    .mul(speed_parallel).rotate(Angle.fromDegrees(swerveTargetAngle));

            // Consider scaling this forward velocity in a non-linear manner
            double speed_forward = kP_VEL_FORWARD * MathUtil.clamp(FORWARD_VEL_SCALING/ta, 0.40, 1.0);
            Vector2D vel_forward = new Vector2D(
                froggyHeading.getCos(),
                froggyHeading.getSin())
                    .mul(speed_forward);

            Pose2d swervePose = swerve.getPose();

            double final_target = angleController.update(
                    Angle.fromDegrees(swerveTargetAngle),
                    Angle.fromRotation2d(swervePose.getRotation()));

            swerve.setControl(swerve.getFieldCentricSwerveRequest()
                    .withVelocityX(linearVelocity.get().x + vel_parallel.x + vel_forward.x)
                    .withVelocityY(linearVelocity.get().y + vel_parallel.y + vel_forward.y)
                    .withRotationalRate(final_target));

            SmartDashboard.putNumber("Vision/TXNC Degrees", txnc.getDegrees());
            SmartDashboard.putNumber("Vision/Current Robot Angle", swervePose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/Target Robot Angle", swerveTargetAngle);

            SmartDashboard.putNumber("Vision/Error of Angle Controller", swerveTargetAngle - swervePose.getRotation().getDegrees());
        } else {
            swerve.setControl(swerve.getFieldCentricSwerveRequest()
                    .withVelocityX(linearVelocity.get().x)
                    .withVelocityY(linearVelocity.get().y)
                    .withRotationalRate(angularVelocity.getAsDouble()));
        }
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