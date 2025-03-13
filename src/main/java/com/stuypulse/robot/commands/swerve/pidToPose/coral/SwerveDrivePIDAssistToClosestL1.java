package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDAssistToClosestL1 extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Gamepad driver;
    
    private final VStream driverLinearVelocity;
    private final IStream driverAngularVelocity;

    private final HolonomicController controller;

    private final FieldObject2d targetPose2d;

    public SwerveDrivePIDAssistToClosestL1(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        driverLinearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        driverAngularVelocity = IStream.create(driver::getRightX)
            .filtered(
                x -> -x,
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * (Turn.MAX_TELEOP_TURN_SPEED.get()),
                new LowPassFilter(Turn.RC));

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
    public void execute() {
        Pose2d targetPose = ReefUtil.ReefFace.getTargetPose(ReefUtil.ReefFace.getNearestReefSide());
        targetPose2d.setPose(Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose));

        controller.update(targetPose, swerve.getPose());

        ChassisSpeeds controllerFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(controller.getOutput(), swerve.getPose().getRotation());
        
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(controllerFieldRelativeSpeeds.vxMetersPerSecond + driverLinearVelocity.get().x)
            .withVelocityY(controllerFieldRelativeSpeeds.vyMetersPerSecond + driverLinearVelocity.get().y)
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