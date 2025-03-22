package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
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

public class SwerveDriveDriveAlignedToBargeClearAllianceSide extends Command {
    private CommandSwerveDrivetrain swerve;
    private Gamepad driver;

    private Controller xController;
    private IStream driverYVelocity;

    private AngleController angleController;

    public SwerveDriveDriveAlignedToBargeClearAllianceSide(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        xController = new PIDController(Gains.Swerve.Alignment.XY.kP, Gains.Swerve.Alignment.XY.kI, Gains.Swerve.Alignment.XY.kD)
            .setSetpointFilter(new MotionProfile(Settings.Swerve.Constraints.MAX_VELOCITY, Settings.Swerve.Constraints.MAX_ACCELERATION));
        
        driverYVelocity = IStream.create(() -> -driver.getLeftX())
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.Drive.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Drive.POWER.get()),
                x -> x * Settings.Driver.Drive.MAX_TELEOP_SPEED.get(),
                new RateLimit(Drive.MAX_TELEOP_ACCEL),
                new LowPassFilter(Drive.RC)
            );
        
        angleController = new AnglePIDController(Gains.Swerve.Alignment.THETA.kP, Gains.Swerve.Alignment.THETA.kI, Gains.Swerve.Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Constraints.MAX_ANGULAR_VELOCITY, Settings.Swerve.Constraints.MAX_ANGULAR_ACCELERATION));
    }

    public void execute() {
        Vector2D targetVelocity = new Vector2D(
                xController.update(Field.LENGTH/2 - Settings.Clearances.CLEARANCE_DISTANCE_FROM_CENTERLINE_BARGE_118, swerve.getPose().getX()), 
                driverYVelocity.get())
            .clamp(Settings.Driver.Drive.MAX_TELEOP_SPEED.get());
        
        swerve.getFieldCentricSwerveRequest()
            .withVelocityX(targetVelocity.x)
            .withVelocityY(targetVelocity.y)
            .withRotationalRate(
                angleController.update(
                    Angle.k180deg, 
                    Angle.fromRotation2d(swerve.getPose().getRotation())
            ));
    }


}
