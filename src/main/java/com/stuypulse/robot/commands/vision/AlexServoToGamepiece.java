package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlexServoToGamepiece extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;
    private final Gamepad driver;

    private final Rotation2d offset;
    private double angleOfGamepiece;
    private Rotation2d robotHeading;

    private final AngleController angleController;
    private final VStream linearVelocity;

    private final static double kP_VEL_PARALLEL = 3.0;
    private final static double kP_VEL_FORWARD = 3.0;

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
        robotHeading = swerve.getPose().getRotation();
        Rotation2d froggyHeading = robotHeading.minus(Rotation2d.fromDegrees(90.0));
        Rotation2d txnc = Rotation2d.fromDegrees(vision.getLastGoodFrame().txncOfHighestArea());
        double ta = vision.getLastGoodFrame().getHighestArea();

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
                Math.cos(Units.degreesToRadians(offset.getDegrees() + 90.0)),
                Math.sin(Units.degreesToRadians(offset.getDegrees() + 90.0)))
                .mul(speed_parallel).rotate(Angle.fromDegrees(swerveTargetAngle));

        double speed_forward = kP_VEL_FORWARD * 1.0 / ta;
        Vector2D vel_forward = new Vector2D(
                froggyHeading.getCos(),
                froggyHeading.getSin())
                .mul(speed_forward);

        double final_target = angleController.update(
                Angle.fromDegrees(swerveTargetAngle),
                Angle.fromRotation2d(swerve.getPose().getRotation()));

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
                .withVelocityX(linearVelocity.get().x + vel_parallel.x + vel_forward.x)
                .withVelocityY(linearVelocity.get().y + vel_parallel.y + vel_forward.y)
                .withRotationalRate(final_target));

        SmartDashboard.putNumber("Vision/TXNC Degrees", txnc.getDegrees());
        SmartDashboard.putNumber("Vision/Current Robot Angle", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/Target Robot Angle", swerveTargetAngle);
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
