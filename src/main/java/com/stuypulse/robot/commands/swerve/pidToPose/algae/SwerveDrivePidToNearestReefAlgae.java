package com.stuypulse.robot.commands.swerve.pidToPose.algae;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.util.ReefUtil;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDrivePidToNearestReefAlgae extends SequentialCommandGroup {
    public SwerveDrivePidToNearestReefAlgae(boolean isFrontFacingReef) {
        addCommands(
            new SwerveDrivePIDToPose(() -> ReefUtil.getClosestAlgae().getReadyPose(isFrontFacingReef))
                .withTolerance(Units.inchesToMeters(4.0), Units.inchesToMeters(4.0), Units.degreesToRadians(5)),
            new SwerveDrivePIDToPose(() -> ReefUtil.getClosestAlgae().getTargetPose(isFrontFacingReef)).withoutMotionProfile()
        );
    }
}
