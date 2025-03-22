package com.stuypulse.robot.commands.swerve.driveAligned.barge118;

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
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.stuylib.streams.numbers.filters.RateLimit;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveAlignedToBarge118ClearOppositeAllianceSide extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final IStream driverYVelocity;

    private final Controller xController;
    private final AngleController angleController;

    public SwerveDriveDriveAlignedToBarge118ClearOppositeAllianceSide(Gamepad driver) {
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
                
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Vector2D targetVelocity = new Vector2D(xController.update(Field.LENGTH / 2 + Settings.Clearances.CLEARANCE_DISTANCE_FROM_CENTERLINE_BARGE_118, swerve.getPose().getX()), driverYVelocity.get())
            .clamp(Math.min(Settings.Driver.Drive.MAX_TELEOP_SPEED, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY));
    
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(targetVelocity.x)
            .withVelocityY(targetVelocity.y)
            .withRotationalRate(angleController.update(
                Angle.kZero,
                Angle.fromRotation2d(swerve.getPose().getRotation()))));
    }
}
