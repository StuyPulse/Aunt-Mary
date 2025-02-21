package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VMotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDToNearestBranchWithClearance extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final HolonomicController controller;
    private final BStream isAligned;
    private final IStream velocityError;

    private int level;
    private boolean isScoringFrontSide;

    private final FieldObject2d targetPose2d;

    private Number xTolerance;
    private Number yTolerance;
    private Number thetaTolerance;
    private Number maxVelocityWhenAligned;

    private Pose2d startingPose;
    private Pose2d scorePose;
    private Pose2d readyPose;
    private VStream translationSetpoint;

    public SwerveDrivePIDToNearestBranchWithClearance(int level, boolean isScoringFrontSide) {
        swerve = CommandSwerveDrivetrain.getInstance();

        controller = new HolonomicController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
                .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.MAX_ANGULAR_VELOCITY, Settings.Swerve.Alignment.Constraints.MAX_ANGULAR_ACCELERATION)));

        translationSetpoint = getNewTranslationSetpointGenerator();

        this.level = level;
        this.isScoringFrontSide = isScoringFrontSide;

        targetPose2d = Field.FIELD2D.getObject("Target Pose");

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));

        velocityError = IStream.create(() -> new Translation2d(controller.getError().vxMetersPerSecond, controller.getError().vyMetersPerSecond).getNorm())
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

        xTolerance = Settings.Swerve.Alignment.Tolerances.X_TOLERANCE;
        yTolerance = Settings.Swerve.Alignment.Tolerances.Y_TOLERANCE;
        thetaTolerance = Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE;
        maxVelocityWhenAligned = Settings.Swerve.Alignment.Tolerances.MAX_VELOCITY_WHEN_ALIGNED;

        addRequirements(swerve);
    }

    public SwerveDrivePIDToNearestBranchWithClearance withTolerance(Number x, Number y, Number theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta;
        return this;
    }

    private Pose2d getCurrentTarget() {
        ArmState correspondingArmState;
        ElevatorState correspondingElevatorState;
        switch (level) {
            case 2:
                correspondingArmState = isScoringFrontSide ? ArmState.L2_FRONT : ArmState.L2_BACK;
                correspondingElevatorState = isScoringFrontSide ? ElevatorState.L2_FRONT : ElevatorState.L2_BACK;
                break;
            case 3:
                correspondingArmState = isScoringFrontSide ? ArmState.L3_FRONT : ArmState.L3_BACK;
                correspondingElevatorState = isScoringFrontSide ? ElevatorState.L3_FRONT : ElevatorState.L3_BACK;
                break;
            case 4:
                correspondingArmState = isScoringFrontSide ? ArmState.L4_FRONT : ArmState.L4_BACK;
                correspondingElevatorState = isScoringFrontSide ? ElevatorState.L4_FRONT : ElevatorState.L4_BACK;
                break;
            default:
                correspondingArmState = isScoringFrontSide ? ArmState.L4_FRONT : ArmState.L4_BACK;
                correspondingElevatorState = isScoringFrontSide ? ElevatorState.L4_FRONT : ElevatorState.L4_BACK;
                break;
        }

        return (Arm.getInstance().getState() == correspondingArmState && Arm.getInstance().atTargetAngle() 
            && Elevator.getInstance().getState() == correspondingElevatorState && Elevator.getInstance().atTargetHeight())
            ? scorePose
            : readyPose;
    }

    // the VStream needs to be recreated everytime the command is scheduled to allow the target tranlation to jump to the start of the path
    private VStream getNewTranslationSetpointGenerator() {
        VStream targetTranslationRelativeToStart = VStream.create(() -> new Vector2D(getCurrentTarget().getTranslation()).sub(new Vector2D(startingPose.getTranslation())))
            .filtered(new VMotionProfile(Settings.Swerve.Alignment.Constraints.MAX_VELOCITY, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION));

        return VStream.create(() -> targetTranslationRelativeToStart.get().add(new Vector2D(startingPose.getTranslation())));
    }

    @Override
    public void initialize() {
        startingPose = swerve.getPose();
        scorePose = Field.getClosestBranch().getScorePose(level, isScoringFrontSide);
        readyPose = Field.getClosestBranch().getReadyPose(isScoringFrontSide);
        translationSetpoint = getNewTranslationSetpointGenerator();
    }

    private boolean isAligned() {
        Pose2d robotPose = swerve.getPose();
        return Math.abs(scorePose.getX() - robotPose.getX()) < xTolerance.doubleValue()
            && Math.abs(scorePose.getY() - robotPose.getY()) < yTolerance.doubleValue()
            && Math.abs(scorePose.getRotation().getRadians() - robotPose.getRotation().getRadians()) < thetaTolerance.doubleValue()
            && velocityError.get() < maxVelocityWhenAligned.doubleValue();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(Robot.isBlue() ? getCurrentTarget() : Field.transformToOppositeAlliance(getCurrentTarget()));

        controller.update(new Pose2d(translationSetpoint.get().getTranslation2d(), scorePose.getRotation()), swerve.getPose());
        
        swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(controller.getOutput().vxMetersPerSecond)
            .withVelocityY(controller.getOutput().vyMetersPerSecond)
            .withRotationalRate(controller.getOutput().omegaRadiansPerSecond));
        
        SmartDashboard.putNumber("Alignment/Target x", getCurrentTarget().getX());
        SmartDashboard.putNumber("Alignment/Target y", getCurrentTarget().getY());
        SmartDashboard.putNumber("Alignment/Target angle", getCurrentTarget().getRotation().getDegrees());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m per s)", controller.getOutput().vxMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m per s)", controller.getOutput().vyMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad per s)", controller.getOutput().omegaRadiansPerSecond);

        SmartDashboard.putBoolean("Alignment/Is Aligned", isAligned());
    }

    private boolean currentTargetIsScorePose() {
        return getCurrentTarget().getTranslation().minus(scorePose.getTranslation()).getNorm() < 0.01;
    }

    @Override
    public boolean isFinished() {
        return isAligned.get() && currentTargetIsScorePose();
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