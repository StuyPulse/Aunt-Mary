package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.stuylib.streams.numbers.filters.RateLimit;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDToPose extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final VStream linearVelocity;
    private final IStream angularVelocity;

    private final HolonomicController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;
    private final IStream velocityError;

    private final FieldObject2d targetPose2d;

    private Number xTolerance;
    private Number yTolerance;
    private Number thetaTolerance;
    private Number maxVelocityWhenAligned;

    private Pose2d targetPose;

    public SwerveDrivePIDToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDrivePIDToPose(Supplier<Pose2d> poseSupplier) {
        swerve = CommandSwerveDrivetrain.getInstance();

        controller = new HolonomicController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD));

        linearVelocity = VStream.create(() -> new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond))
            .filtered(
                x -> x.clamp(Settings.Swerve.Alignment.MAX_VELOCITY.get()),
                new VRateLimit(Settings.Swerve.Alignment.MAX_ACCELERATION));

        angularVelocity = IStream.create(() -> controller.getOutput().omegaRadiansPerSecond)
            .filtered(
                x -> -x,
                x -> SLMath.deadband(x, Settings.Swerve.Alignment.MAX_ANGULAR_VELOCITY.get()),
                new RateLimit(Settings.Swerve.Alignment.MAX_ANGULAR_ACCELERATION.get()));

        this.poseSupplier = poseSupplier;

        targetPose2d = Field.FIELD2D.getObject("Target Pose");

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.ALIGNMENT_DEBOUNCE));

        velocityError = IStream.create(() -> new Translation2d(controller.getError().vxMetersPerSecond, controller.getError().vyMetersPerSecond).getNorm())
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

        xTolerance = Settings.Swerve.Alignment.X_TOLERANCE;
        yTolerance = Settings.Swerve.Alignment.Y_TOLERANCE;
        thetaTolerance = Settings.Swerve.Alignment.THETA_TOLERANCE;
        maxVelocityWhenAligned = Settings.Swerve.Alignment.MAX_VELOCITY_WHEN_ALIGNED;

        addRequirements(swerve);
    }

    public SwerveDrivePIDToPose withTranslationConstants(PIDConstants pid) {
        controller.setTranslationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }
    
    public SwerveDrivePIDToPose withRotationConstants(PIDConstants pid) {
        controller.setRotationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }

    public SwerveDrivePIDToPose withTranslationConstants(double p, double i, double d) {
        controller.setTranslationConstants(p, i, d);
        return this;
    }
    
    public SwerveDrivePIDToPose withRotationConstants(double p, double i, double d) {
        controller.setRotationConstants(p, i, d);
        return this;
    }

    public SwerveDrivePIDToPose withTolerance(Number x, Number y, Number theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta;
        return this;
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();
    }

    private boolean isAligned() {
        return controller.isDone(xTolerance.doubleValue(), yTolerance.doubleValue(), Math.toDegrees(thetaTolerance.doubleValue()))
            && velocityError.get() < maxVelocityWhenAligned.doubleValue();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose));

        SmartDashboard.putNumber("Alignment/Target x", targetPose.getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Alignment/Is Aligned", isAligned());

        controller.update(targetPose, swerve.getPose());
        
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(linearVelocity.get().x)
            .withVelocityY(linearVelocity.get().y)
            .withRotationalRate(angularVelocity.get()));
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
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