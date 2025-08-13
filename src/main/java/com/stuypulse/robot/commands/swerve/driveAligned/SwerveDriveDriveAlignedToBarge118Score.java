package com.stuypulse.robot.commands.swerve.driveAligned;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.stuylib.streams.numbers.filters.RateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveDriveAlignedToBarge118Score extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final IStream driverYVelocity;

    private final Controller xController;
    private final AngleController angleController;

    private boolean isAngled;

    public SwerveDriveDriveAlignedToBarge118Score(CommandXboxController driver, boolean isAngled) {
        swerve = CommandSwerveDrivetrain.getInstance();

        driverYVelocity = IStream.create(() -> -driver.getLeftX())
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.Drive.DEADBAND),
                x -> SLMath.spow(x, Settings.Driver.Drive.POWER),
                x -> x * Settings.Driver.Drive.MAX_TELEOP_SPEED,
                new RateLimit(Drive.MAX_TELEOP_ACCEL),
                new LowPassFilter(Drive.RC));

        xController = new PIDController(Gains.Swerve.Alignment.XY.kP, Gains.Swerve.Alignment.XY.kI, Gains.Swerve.Alignment.XY.kD)
            .setSetpointFilter(new MotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION));

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFUALT_MAX_ANGULAR_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION));
               
        this.isAngled = isAngled;
        addRequirements(swerve);
    }


    private double getTargetX() {
        return CommandSwerveDrivetrain.getInstance().isOnAllianceSide()
            ? Field.LENGTH / 2 - (Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118)
            : Field.LENGTH / 2 + (Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118);
    }

    private Angle getTargetAngle() {
        return (isAngled ? 
            CommandSwerveDrivetrain.getInstance().isOnAllianceSide()
            ? Angle.k180deg.addDegrees(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_118.getDegrees())
            : Angle.kZero.subDegrees(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_118.getDegrees())
        :
            CommandSwerveDrivetrain.getInstance().isOnAllianceSide()
            ? Angle.k180deg
            : Angle.kZero);
    }

    @Override
    public void execute() {
        Translation2d targetVelocity = new Translation2d(xController.update(getTargetX(), swerve.getPose().getX()), driverYVelocity.get());
    
        double maxVelocity = Math.min(Settings.Driver.Drive.MAX_TELEOP_SPEED, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY);
        double currentVelocity = targetVelocity.getDistance(new Translation2d(0, 0));
        
        targetVelocity = 
            (currentVelocity>maxVelocity) ? 
            new Translation2d(targetVelocity.getX() * (maxVelocity/currentVelocity), targetVelocity.getY() * (maxVelocity/currentVelocity)): targetVelocity;
    
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(targetVelocity.getX())
            .withVelocityY(targetVelocity.getY())
            .withRotationalRate(angleController.update(
                getTargetAngle(),
                Angle.fromRotation2d(swerve.getPose().getRotation()))));
    }
}